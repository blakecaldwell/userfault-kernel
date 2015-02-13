/*
 *  mm/userfaultfd.c
 *
 *  Copyright (C) 2015  Red Hat, Inc.
 *
 *  This work is licensed under the terms of the GNU GPL, version 2. See
 *  the COPYING file in the top-level directory.
 */

#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/rmap.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/userfaultfd_k.h>
#include <linux/mmu_notifier.h>
#include <asm/tlbflush.h>
#include "internal.h"

static int mcopy_atomic_pte(struct mm_struct *dst_mm,
			    pmd_t *dst_pmd,
			    struct vm_area_struct *dst_vma,
			    unsigned long dst_addr,
			    unsigned long src_addr,
			    struct page **pagep)
{
	struct mem_cgroup *memcg;
	pte_t _dst_pte, *dst_pte;
	spinlock_t *ptl;
	void *page_kaddr;
	int ret;
	struct page *page;

	if (!*pagep) {
		ret = -ENOMEM;
		page = alloc_page_vma(GFP_HIGHUSER_MOVABLE, dst_vma, dst_addr);
		if (!page)
			goto out;

		page_kaddr = kmap_atomic(page);
		ret = copy_from_user(page_kaddr,
				     (const void __user *) src_addr,
				     PAGE_SIZE);
		kunmap_atomic(page_kaddr);

		/* fallback to copy_from_user outside mmap_sem */
		if (unlikely(ret)) {
			ret = -EFAULT;
			*pagep = page;
			/* don't free the page */
			goto out;
		}
	} else {
		page = *pagep;
		*pagep = NULL;
	}

	/*
	 * The memory barrier inside __SetPageUptodate makes sure that
	 * preceeding stores to the page contents become visible before
	 * the set_pte_at() write.
	 */
	__SetPageUptodate(page);

	ret = -ENOMEM;
	if (mem_cgroup_try_charge(page, dst_mm, GFP_KERNEL, &memcg))
		goto out_release;

	_dst_pte = mk_pte(page, dst_vma->vm_page_prot);
	if (dst_vma->vm_flags & VM_WRITE)
		_dst_pte = pte_mkwrite(pte_mkdirty(_dst_pte));

	ret = -EEXIST;
	dst_pte = pte_offset_map_lock(dst_mm, dst_pmd, dst_addr, &ptl);
	if (!pte_none(*dst_pte))
		goto out_release_uncharge_unlock;

	inc_mm_counter(dst_mm, MM_ANONPAGES);
	page_add_new_anon_rmap(page, dst_vma, dst_addr);
	mem_cgroup_commit_charge(page, memcg, false);
	lru_cache_add_active_or_unevictable(page, dst_vma);

	set_pte_at(dst_mm, dst_addr, dst_pte, _dst_pte);

	/* No need to invalidate - it was non-present before */
	update_mmu_cache(dst_vma, dst_addr, dst_pte);

	pte_unmap_unlock(dst_pte, ptl);
	ret = 0;
out:
	return ret;
out_release_uncharge_unlock:
	pte_unmap_unlock(dst_pte, ptl);
	mem_cgroup_cancel_charge(page, memcg);
out_release:
	page_cache_release(page);
	goto out;
}

static int mfill_zeropage_pte(struct mm_struct *dst_mm,
			      pmd_t *dst_pmd,
			      struct vm_area_struct *dst_vma,
			      unsigned long dst_addr)
{
	pte_t _dst_pte, *dst_pte;
	spinlock_t *ptl;
	int ret;

	_dst_pte = pte_mkspecial(pfn_pte(my_zero_pfn(dst_addr),
					 dst_vma->vm_page_prot));
	ret = -EEXIST;
	dst_pte = pte_offset_map_lock(dst_mm, dst_pmd, dst_addr, &ptl);
	if (!pte_none(*dst_pte))
		goto out_unlock;
	set_pte_at(dst_mm, dst_addr, dst_pte, _dst_pte);
	/* No need to invalidate - it was non-present before */
	update_mmu_cache(dst_vma, dst_addr, dst_pte);
	ret = 0;
out_unlock:
	pte_unmap_unlock(dst_pte, ptl);
	return ret;
}

static pmd_t *mm_alloc_pmd(struct mm_struct *mm, unsigned long address)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd = NULL;

	pgd = pgd_offset(mm, address);
	pud = pud_alloc(mm, pgd, address);
	if (pud)
		/*
		 * Note that we didn't run this because the pmd was
		 * missing, the *pmd may be already established and in
		 * turn it may also be a trans_huge_pmd.
		 */
		pmd = pmd_alloc(mm, pud, address);
	return pmd;
}

static __always_inline ssize_t __mcopy_atomic(struct mm_struct *dst_mm,
					      unsigned long dst_start,
					      unsigned long src_start,
					      unsigned long len,
					      bool zeropage)
{
	struct vm_area_struct *dst_vma;
	ssize_t err;
	pmd_t *dst_pmd;
	unsigned long src_addr, dst_addr;
	long copied;
	struct page *page;

	/*
	 * Sanitize the command parameters:
	 */
	BUG_ON(dst_start & ~PAGE_MASK);
	BUG_ON(len & ~PAGE_MASK);

	/* Does the address range wrap, or is the span zero-sized? */
	BUG_ON(src_start + len <= src_start);
	BUG_ON(dst_start + len <= dst_start);

	src_addr = src_start;
	dst_addr = dst_start;
	copied = 0;
	page = NULL;
retry:
	down_read(&dst_mm->mmap_sem);

	/*
	 * Make sure the vma is not shared, that the dst range is
	 * both valid and fully within a single existing vma.
	 */
	err = -EINVAL;
	dst_vma = find_vma(dst_mm, dst_start);
	if (!dst_vma || (dst_vma->vm_flags & VM_SHARED))
		goto out_unlock;
	if (dst_start < dst_vma->vm_start ||
	    dst_start + len > dst_vma->vm_end)
		goto out_unlock;

	/*
	 * Be strict and only allow __mcopy_atomic on userfaultfd
	 * registered ranges to prevent userland errors going
	 * unnoticed. As far as the VM consistency is concerned, it
	 * would be perfectly safe to remove this check, but there's
	 * no useful usage for __mcopy_atomic ouside of userfaultfd
	 * registered ranges. This is after all why these are ioctls
	 * belonging to the userfaultfd and not syscalls.
	 */
	if (!dst_vma->vm_userfaultfd_ctx.ctx)
		goto out_unlock;

	/*
	 * FIXME: only allow copying on anonymous vmas, tmpfs should
	 * be added.
	 */
	if (dst_vma->vm_ops)
		goto out_unlock;

	/*
	 * Ensure the dst_vma has a anon_vma or this page
	 * would get a NULL anon_vma when moved in the
	 * dst_vma.
	 */
	err = -ENOMEM;
	if (unlikely(anon_vma_prepare(dst_vma)))
		goto out_unlock;

	while (src_addr < src_start + len) {
		pmd_t dst_pmdval;

		BUG_ON(dst_addr >= dst_start + len);

		dst_pmd = mm_alloc_pmd(dst_mm, dst_addr);
		if (unlikely(!dst_pmd)) {
			err = -ENOMEM;
			break;
		}

		dst_pmdval = pmd_read_atomic(dst_pmd);
		/*
		 * If the dst_pmd is mapped as THP don't
		 * override it and just be strict.
		 */
		if (unlikely(pmd_trans_huge(dst_pmdval))) {
			err = -EEXIST;
			break;
		}
		if (unlikely(pmd_none(dst_pmdval)) &&
		    unlikely(__pte_alloc(dst_mm, dst_vma, dst_pmd,
					 dst_addr))) {
			err = -ENOMEM;
			break;
		}
		/* If an huge pmd materialized from under us fail */
		if (unlikely(pmd_trans_huge(*dst_pmd))) {
			err = -EFAULT;
			break;
		}

		BUG_ON(pmd_none(*dst_pmd));
		BUG_ON(pmd_trans_huge(*dst_pmd));

		if (!zeropage)
			err = mcopy_atomic_pte(dst_mm, dst_pmd, dst_vma,
					       dst_addr, src_addr, &page);
		else
			err = mfill_zeropage_pte(dst_mm, dst_pmd, dst_vma,
						 dst_addr);

		cond_resched();

		if (unlikely(err == -EFAULT)) {
			void *page_kaddr;

			up_read(&dst_mm->mmap_sem);
			BUG_ON(!page);

			page_kaddr = kmap(page);
			err = copy_from_user(page_kaddr,
					     (const void __user *) src_addr,
					     PAGE_SIZE);
			kunmap(page);
			if (unlikely(err)) {
				err = -EFAULT;
				goto out;
			}
			goto retry;
		} else
			BUG_ON(page);

		if (!err) {
			dst_addr += PAGE_SIZE;
			src_addr += PAGE_SIZE;
			copied += PAGE_SIZE;

			if (fatal_signal_pending(current))
				err = -EINTR;
		}
		if (err)
			break;
	}

out_unlock:
	up_read(&dst_mm->mmap_sem);
out:
	if (page)
		page_cache_release(page);
	BUG_ON(copied < 0);
	BUG_ON(err > 0);
	BUG_ON(!copied && !err);
	return copied ? copied : err;
}

ssize_t mcopy_atomic(struct mm_struct *dst_mm, unsigned long dst_start,
		     unsigned long src_start, unsigned long len)
{
	return __mcopy_atomic(dst_mm, dst_start, src_start, len, false);
}

ssize_t mfill_zeropage(struct mm_struct *dst_mm, unsigned long start,
		       unsigned long len)
{
	return __mcopy_atomic(dst_mm, start, 0, len, true);
}

void double_pt_lock(spinlock_t *ptl1,
		    spinlock_t *ptl2)
	__acquires(ptl1)
	__acquires(ptl2)
{
	spinlock_t *ptl_tmp;

	if (ptl1 > ptl2) {
		/* exchange ptl1 and ptl2 */
		ptl_tmp = ptl1;
		ptl1 = ptl2;
		ptl2 = ptl_tmp;
	}
	/* lock in virtual address order to avoid lock inversion */
	spin_lock(ptl1);
	if (ptl1 != ptl2)
		spin_lock_nested(ptl2, SINGLE_DEPTH_NESTING);
	else
		__acquire(ptl2);
}

void double_pt_unlock(spinlock_t *ptl1,
		      spinlock_t *ptl2)
	__releases(ptl1)
	__releases(ptl2)
{
	spin_unlock(ptl1);
	if (ptl1 != ptl2)
		spin_unlock(ptl2);
	else
		__release(ptl2);
}

/*
 * The mmap_sem for reading is held by the caller. Just move the page
 * from src_pmd to dst_pmd if possible, and return true if succeeded
 * in moving the page.
 */
static int remap_pages_pte(struct mm_struct *dst_mm,
			   struct mm_struct *src_mm,
			   pte_t *dst_pte, pte_t *src_pte, pmd_t *src_pmd,
			   struct vm_area_struct *dst_vma,
			   struct vm_area_struct *src_vma,
			   unsigned long dst_addr,
			   unsigned long src_addr,
			   spinlock_t *dst_ptl,
			   spinlock_t *src_ptl,
			   __u64 mode)
{
	struct page *src_page;
	swp_entry_t entry;
	pte_t orig_src_pte, orig_dst_pte;
	struct anon_vma *src_anon_vma, *dst_anon_vma;

	spin_lock(dst_ptl);
	orig_dst_pte = *dst_pte;
	spin_unlock(dst_ptl);
	if (!pte_none(orig_dst_pte))
		return -EEXIST;

	spin_lock(src_ptl);
	orig_src_pte = *src_pte;
	spin_unlock(src_ptl);
	if (pte_none(orig_src_pte)) {
		if (!(mode & UFFDIO_REMAP_MODE_ALLOW_SRC_HOLES))
			return -ENOENT;
		else
			/* nothing to do to remap an hole */
			return 0;
	}

	if (pte_present(orig_src_pte)) {
		/*
		 * Pin the page while holding the lock to be sure the
		 * page isn't freed under us
		 */
		spin_lock(src_ptl);
		if (!pte_same(orig_src_pte, *src_pte)) {
			spin_unlock(src_ptl);
			return -EAGAIN;
		}
		src_page = vm_normal_page(src_vma, src_addr, orig_src_pte);
		if (!src_page || !PageAnon(src_page) ||
		    page_mapcount(src_page) != 1) {
			spin_unlock(src_ptl);
			return -EBUSY;
		}

		get_page(src_page);
		spin_unlock(src_ptl);

		/* block all concurrent rmap walks */
		lock_page(src_page);

		/*
		 * page_referenced_anon walks the anon_vma chain
		 * without the page lock. Serialize against it with
		 * the anon_vma lock, the page lock is not enough.
		 */
		src_anon_vma = page_get_anon_vma(src_page);
		if (!src_anon_vma) {
			/* page was unmapped from under us */
			unlock_page(src_page);
			put_page(src_page);
			return -EAGAIN;
		}
		anon_vma_lock_write(src_anon_vma);

		double_pt_lock(dst_ptl, src_ptl);

		if (!pte_same(*src_pte, orig_src_pte) ||
		    !pte_same(*dst_pte, orig_dst_pte) ||
		    page_mapcount(src_page) != 1) {
			double_pt_unlock(dst_ptl, src_ptl);
			anon_vma_unlock_write(src_anon_vma);
			put_anon_vma(src_anon_vma);
			unlock_page(src_page);
			put_page(src_page);
			return -EAGAIN;
		}

		BUG_ON(!PageAnon(src_page));
		/* the PT lock is enough to keep the page pinned now */
		put_page(src_page);

		dst_anon_vma = (void *) dst_vma->anon_vma + PAGE_MAPPING_ANON;
		ACCESS_ONCE(src_page->mapping) = ((struct address_space *)
						  dst_anon_vma);
		ACCESS_ONCE(src_page->index) = linear_page_index(dst_vma,
								 dst_addr);

		if (!pte_same(ptep_clear_flush(src_vma, src_addr, src_pte),
			      orig_src_pte))
			BUG();

		orig_dst_pte = mk_pte(src_page, dst_vma->vm_page_prot);
		orig_dst_pte = maybe_mkwrite(pte_mkdirty(orig_dst_pte),
					     dst_vma);

		set_pte_at(dst_mm, dst_addr, dst_pte, orig_dst_pte);

		if (dst_mm != src_mm) {
			inc_mm_counter(dst_mm, MM_ANONPAGES);
			dec_mm_counter(src_mm, MM_ANONPAGES);
		}

		double_pt_unlock(dst_ptl, src_ptl);

		anon_vma_unlock_write(src_anon_vma);
		put_anon_vma(src_anon_vma);

		/* unblock rmap walks */
		unlock_page(src_page);

		mmu_notifier_invalidate_page(src_mm, src_addr);
	} else {
		entry = pte_to_swp_entry(orig_src_pte);
		if (non_swap_entry(entry)) {
			if (is_migration_entry(entry)) {
				migration_entry_wait(src_mm, src_pmd,
						     src_addr);
				return -EAGAIN;
			}
			return -EFAULT;
		}

		if (swp_swapcount(entry) != 1) {
			printk("returning EBUSY because of swp_swapcount\n");
			return -EBUSY;
		}

		double_pt_lock(dst_ptl, src_ptl);

		if (!pte_same(*src_pte, orig_src_pte) ||
		    !pte_same(*dst_pte, orig_dst_pte) ||
		    swp_swapcount(entry) != 1) {
			printk("returning EAGAIN because of swp_swapcount\n");
			double_pt_unlock(dst_ptl, src_ptl);
			return -EAGAIN;
		}

		if (pte_val(ptep_get_and_clear(src_mm, src_addr, src_pte)) !=
		    pte_val(orig_src_pte))
			BUG();
		set_pte_at(dst_mm, dst_addr, dst_pte, orig_src_pte);

		if (dst_mm != src_mm) {
			inc_mm_counter(dst_mm, MM_ANONPAGES);
			dec_mm_counter(src_mm, MM_ANONPAGES);
		}

		double_pt_unlock(dst_ptl, src_ptl);
	}

	return 0;
}

/**
 * remap_pages - remap arbitrary anonymous pages of an existing vma
 * @dst_start: start of the destination virtual memory range
 * @src_start: start of the source virtual memory range
 * @len: length of the virtual memory range
 *
 * remap_pages() remaps arbitrary anonymous pages atomically in zero
 * copy. It only works on non shared anonymous pages because those can
 * be relocated without generating non linear anon_vmas in the rmap
 * code.
 *
 * It is the ideal mechanism to handle userspace page faults. Normally
 * the destination vma will have VM_USERFAULT set with
 * madvise(MADV_USERFAULT) while the source vma will have VM_DONTCOPY
 * set with madvise(MADV_DONTFORK).
 *
 * The thread receiving the page during the userland page fault
 * (MADV_USERFAULT) will receive the faulting page in the source vma
 * through the network, storage or any other I/O device (MADV_DONTFORK
 * in the source vma avoids remap_pages() to fail with -EBUSY if the
 * process forks before remap_pages() is called), then it will call
 * remap_pages() to map the page in the faulting address in the
 * destination vma.
 *
 * This userfaultfd command works purely via pagetables, so it's the
 * most efficient way to move physical non shared anonymous pages
 * across different virtual addresses. Unlike mremap()/mmap()/munmap()
 * it does not create any new vmas. The mapping in the destination
 * address is atomic.
 *
 * It only works if the vma protection bits are identical from the
 * source and destination vma.
 *
 * It can remap non shared anonymous pages within the same vma too.
 *
 * If the source virtual memory range has any unmapped holes, or if
 * the destination virtual memory range is not a whole unmapped hole,
 * remap_pages() will fail respectively with -ENOENT or -EEXIST. This
 * provides a very strict behavior to avoid any chance of memory
 * corruption going unnoticed if there are userland race
 * conditions. Only one thread should resolve the userland page fault
 * at any given time for any given faulting address. This means that
 * if two threads try to both call remap_pages() on the same
 * destination address at the same time, the second thread will get an
 * explicit error from this command.
 *
 * The command retval will return "len" is succesful. The command
 * however can be interrupted by fatal signals or errors. If
 * interrupted it will return the number of bytes successfully
 * remapped before the interruption if any, or the negative error if
 * none. It will never return zero. Either it will return an error or
 * an amount of bytes successfully moved. If the retval reports a
 * "short" remap, the remap_pages() command should be repeated by
 * userland with src+retval, dst+reval, len-retval if it wants to know
 * about the error that interrupted it.
 *
 * The UFFDIO_REMAP_MODE_ALLOW_SRC_HOLES flag can be specified to
 * prevent -ENOENT errors to materialize if there are holes in the
 * source virtual range that is being remapped. The holes will be
 * accounted as successfully remapped in the retval of the
 * command. This is mostly useful to remap hugepage naturally aligned
 * virtual regions without knowing if there are transparent hugepage
 * in the regions or not, but preventing the risk of having to split
 * the hugepmd during the remap.
 *
 * If there's any rmap walk that is taking the anon_vma locks without
 * first obtaining the page lock (for example split_huge_page and
 * page_referenced_anon), they will have to verify if the
 * page->mapping has changed after taking the anon_vma lock. If it
 * changed they should release the lock and retry obtaining a new
 * anon_vma, because it means the anon_vma was changed by
 * remap_pages() before the lock could be obtained. This is the only
 * additional complexity added to the rmap code to provide this
 * anonymous page remapping functionality.
 */
ssize_t remap_pages(struct mm_struct *dst_mm, struct mm_struct *src_mm,
		    unsigned long dst_start, unsigned long src_start,
		    unsigned long len, __u64 mode)
{
	struct vm_area_struct *src_vma, *dst_vma;
	long err = -EINVAL;
	pmd_t *src_pmd, *dst_pmd;
	pte_t *src_pte, *dst_pte;
	spinlock_t *dst_ptl, *src_ptl;
	unsigned long src_addr, dst_addr;
	int thp_aligned = -1;
	ssize_t moved = 0;

	/*
	 * Sanitize the command parameters:
	 */
	BUG_ON(src_start & ~PAGE_MASK);
	BUG_ON(dst_start & ~PAGE_MASK);
	BUG_ON(len & ~PAGE_MASK);

	/* Does the address range wrap, or is the span zero-sized? */
	BUG_ON(src_start + len <= src_start);
	BUG_ON(dst_start + len <= dst_start);

	/*
	 * Because these are read sempahores there's no risk of lock
	 * inversion.
	 */
	down_read(&dst_mm->mmap_sem);
	if (dst_mm != src_mm)
		down_read(&src_mm->mmap_sem);

	/*
	 * Make sure the vma is not shared, that the src and dst remap
	 * ranges are both valid and fully within a single existing
	 * vma.
	 */
	src_vma = find_vma(src_mm, src_start);
	if (!src_vma || (src_vma->vm_flags & VM_SHARED))
		goto out;
	if (src_start < src_vma->vm_start ||
	    src_start + len > src_vma->vm_end)
		goto out;

	dst_vma = find_vma(dst_mm, dst_start);
	if (!dst_vma || (dst_vma->vm_flags & VM_SHARED))
		goto out;
	if (dst_start < dst_vma->vm_start ||
	    dst_start + len > dst_vma->vm_end)
		goto out;

	if (pgprot_val(src_vma->vm_page_prot) !=
	    pgprot_val(dst_vma->vm_page_prot))
		goto out;

	/* only allow remapping if both are mlocked or both aren't */
	if ((src_vma->vm_flags & VM_LOCKED) ^ (dst_vma->vm_flags & VM_LOCKED))
		goto out;

	/*
	 * Be strict and only allow remap_pages if either the src or
	 * dst range is registered in the userfaultfd to prevent
	 * userland errors going unnoticed. As far as the VM
	 * consistency is concerned, it would be perfectly safe to
	 * remove this check, but there's no useful usage for
	 * remap_pages ouside of userfaultfd registered ranges. This
	 * is after all why it is an ioctl belonging to the
	 * userfaultfd and not a syscall.
	 *
	 * Allow both vmas to be registered in the userfaultfd, just
	 * in case somebody finds a way to make such a case useful.
	 * Normally only one of the two vmas would be registered in
	 * the userfaultfd.
	 */
	if (!dst_vma->vm_userfaultfd_ctx.ctx &&
	    !src_vma->vm_userfaultfd_ctx.ctx)
		goto out;

	/*
	 * FIXME: only allow remapping across anonymous vmas,
	 * tmpfs should be added.
	 */
	if (src_vma->vm_ops || dst_vma->vm_ops)
		goto out;

	/*
	 * Ensure the dst_vma has a anon_vma or this page
	 * would get a NULL anon_vma when moved in the
	 * dst_vma.
	 */
	err = -ENOMEM;
	if (unlikely(anon_vma_prepare(dst_vma)))
		goto out;

	for (src_addr = src_start, dst_addr = dst_start;
	     src_addr < src_start + len; ) {
		spinlock_t *ptl;
		pmd_t dst_pmdval;
		BUG_ON(dst_addr >= dst_start + len);
		src_pmd = mm_find_pmd(src_mm, src_addr);
		if (unlikely(!src_pmd)) {
			if (!(mode & UFFDIO_REMAP_MODE_ALLOW_SRC_HOLES)) {
				err = -ENOENT;
				break;
			} else {
				src_pmd = mm_alloc_pmd(src_mm, src_addr);
				if (unlikely(!src_pmd)) {
					err = -ENOMEM;
					break;
				}
			}
		}
		dst_pmd = mm_alloc_pmd(dst_mm, dst_addr);
		if (unlikely(!dst_pmd)) {
			err = -ENOMEM;
			break;
		}

		dst_pmdval = pmd_read_atomic(dst_pmd);
		/*
		 * If the dst_pmd is mapped as THP don't
		 * override it and just be strict.
		 */
		if (unlikely(pmd_trans_huge(dst_pmdval))) {
			err = -EEXIST;
			break;
		}
		if (pmd_trans_huge_lock(src_pmd, src_vma, &ptl) == 1) {
			/*
			 * Check if we can move the pmd without
			 * splitting it. First check the address
			 * alignment to be the same in src/dst.  These
			 * checks don't actually need the PT lock but
			 * it's good to do it here to optimize this
			 * block away at build time if
			 * CONFIG_TRANSPARENT_HUGEPAGE is not set.
			 */
			if (thp_aligned == -1)
				thp_aligned = ((src_addr & ~HPAGE_PMD_MASK) ==
					       (dst_addr & ~HPAGE_PMD_MASK));
			if (!thp_aligned || (src_addr & ~HPAGE_PMD_MASK) ||
			    !pmd_none(dst_pmdval) ||
			    src_start + len - src_addr < HPAGE_PMD_SIZE) {
				spin_unlock(ptl);
				/* Fall through */
				split_huge_page_pmd(src_vma, src_addr,
						    src_pmd);
			} else {
				BUG_ON(dst_addr & ~HPAGE_PMD_MASK);
				err = remap_pages_huge_pmd(dst_mm,
							   src_mm,
							   dst_pmd,
							   src_pmd,
							   dst_pmdval,
							   dst_vma,
							   src_vma,
							   dst_addr,
							   src_addr);
				cond_resched();

				if (!err) {
					dst_addr += HPAGE_PMD_SIZE;
					src_addr += HPAGE_PMD_SIZE;
					moved += HPAGE_PMD_SIZE;
				}

				if ((!err || err == -EAGAIN) &&
				    fatal_signal_pending(current))
					err = -EINTR;

				if (err && err != -EAGAIN)
					break;

				continue;
			}
		}

		if (pmd_none(*src_pmd)) {
			if (!(mode & UFFDIO_REMAP_MODE_ALLOW_SRC_HOLES)) {
				err = -ENOENT;
				break;
			} else {
				if (unlikely(__pte_alloc(src_mm, src_vma,
							 src_pmd, src_addr))) {
					err = -ENOMEM;
					break;
				}
			}
		}

		/*
		 * We held the mmap_sem for reading so MADV_DONTNEED
		 * can zap transparent huge pages under us, or the
		 * transparent huge page fault can establish new
		 * transparent huge pages under us.
		 */
		if (unlikely(pmd_trans_unstable(src_pmd))) {
			err = -EFAULT;
			break;
		}

		if (unlikely(pmd_none(dst_pmdval)) &&
		    unlikely(__pte_alloc(dst_mm, dst_vma, dst_pmd,
					 dst_addr))) {
			err = -ENOMEM;
			break;
		}
		/* If an huge pmd materialized from under us fail */
		if (unlikely(pmd_trans_huge(*dst_pmd))) {
			err = -EFAULT;
			break;
		}

		BUG_ON(pmd_none(*dst_pmd));
		BUG_ON(pmd_none(*src_pmd));
		BUG_ON(pmd_trans_huge(*dst_pmd));
		BUG_ON(pmd_trans_huge(*src_pmd));

		dst_pte = pte_offset_map(dst_pmd, dst_addr);
		src_pte = pte_offset_map(src_pmd, src_addr);
		dst_ptl = pte_lockptr(dst_mm, dst_pmd);
		src_ptl = pte_lockptr(src_mm, src_pmd);

		err = remap_pages_pte(dst_mm, src_mm,
				      dst_pte, src_pte, src_pmd,
				      dst_vma, src_vma,
				      dst_addr, src_addr,
				      dst_ptl, src_ptl, mode);

		pte_unmap(dst_pte);
		pte_unmap(src_pte);
		cond_resched();

		if (!err) {
			dst_addr += PAGE_SIZE;
			src_addr += PAGE_SIZE;
			moved += PAGE_SIZE;
		}

		if ((!err || err == -EAGAIN) &&
		    fatal_signal_pending(current))
			err = -EINTR;

		if (err && err != -EAGAIN)
			break;
	}

out:
	up_read(&dst_mm->mmap_sem);
	if (dst_mm != src_mm)
		up_read(&src_mm->mmap_sem);
	BUG_ON(moved < 0);
	BUG_ON(err > 0);
	BUG_ON(!moved && !err);
	return moved ? moved : err;
}
