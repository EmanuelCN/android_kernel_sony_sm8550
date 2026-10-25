// SPDX-License-Identifier: GPL-2.0-or-later

#define pr_fmt(fmt) "zram: zstd: " fmt

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/zstd.h>

#include "backend_zstd.h"

#define ZRAM_ZSTD_DEFAULT_LEVEL 3

struct zstd_ctx {
	void *ctx;
	void *mem;
};

struct zstd_params {
	ZSTD_customMem custom_mem;
	ZSTD_CDict *cdict;
	ZSTD_DDict *ddict;
	ZSTD_parameters cprm;
};

/*
 * For C/D dictionaries we need to provide zstd with ZSTD_customMem,
 * which zstd uses internally to allocate/free memory when needed.
 *
 * zstd allocations (C/D dictionaries and contexts) are large and
 * long-lived.  Don't use kvzalloc() for them: when enough physically
 * contiguous memory is available it serves them from kmalloc(), which
 * rounds them up to a power-of-two number of pages.
 */
static void *zstd_custom_alloc(void *opaque, size_t size)
{
	if (size <= PAGE_SIZE)
		return kzalloc(size, GFP_NOIO | __GFP_NOWARN);
	return __vmalloc(size, GFP_NOIO | __GFP_NOWARN | __GFP_ZERO);
}

static void zstd_custom_free(void *opaque, void *address)
{
	kvfree(address);
}

static void zstd_release_params(struct zcomp_params *params)
{
	struct zstd_params *zp = params->drv_data;

	params->drv_data = NULL;
	if (!zp)
		return;

	ZSTD_freeCDict(zp->cdict);
	ZSTD_freeDDict(zp->ddict);
	kfree(zp);
}

static int zstd_setup_params(struct zcomp_params *params)
{
	ZSTD_parameters prm;
	struct zstd_params *zp;

	zp = kzalloc(sizeof(*zp), GFP_KERNEL);
	if (!zp)
		return -ENOMEM;

	params->drv_data = zp;
	if (params->level == ZCOMP_PARAM_NO_LEVEL)
		params->level = ZRAM_ZSTD_DEFAULT_LEVEL;

	zp->cprm = ZSTD_getParams(params->level, PAGE_SIZE, 0);

	zp->custom_mem.customAlloc = zstd_custom_alloc;
	zp->custom_mem.customFree = zstd_custom_free;

	if (!params->dict || !params->dict_sz)
		return 0;

	prm = ZSTD_getParams(params->level, PAGE_SIZE, params->dict_sz);
	zp->cdict = ZSTD_createCDict_byReference(params->dict, params->dict_sz,
						prm, zp->custom_mem);
	if (!zp->cdict)
		goto error;

	zp->ddict = ZSTD_createDDict_byReference(params->dict,
						  params->dict_sz,
						  zp->custom_mem);
	if (!zp->ddict)
		goto error;

	return 0;

error:
	zstd_release_params(params);
	return -EINVAL;
}

static void zstd_destroy_cctx(struct zcomp_ctx *ctx)
{
	struct zstd_ctx *zctx = ctx->context;

	if (!zctx)
		return;

	/* Static contexts are embedded in the workspace. */
	if (zctx->mem)
		vfree(zctx->mem);
	else
		ZSTD_freeCCtx(zctx->ctx);

	kfree(zctx);
}

static int zstd_create_cctx(struct zcomp_params *params, struct zcomp_ctx *ctx)
{
	struct zstd_params *zp = params->drv_data;
	struct zstd_ctx *zctx;

	zctx = kzalloc(sizeof(*zctx), GFP_KERNEL);
	if (!zctx)
		return -ENOMEM;

	ctx->context = zctx;
	if (!zp->cdict) {
		size_t size = ZSTD_CCtxWorkspaceBound(zp->cprm.cParams);

		zctx->mem = vzalloc(size);
		if (!zctx->mem)
			goto error;
		zctx->ctx = ZSTD_initCCtx(zctx->mem, size);
	} else {
		zctx->ctx = ZSTD_createCCtx_advanced(zp->custom_mem);
	}
	if (!zctx->ctx)
		goto error;

	return 0;

error:
	zstd_destroy_cctx(ctx);
	return -ENOMEM;
}

static void zstd_destroy_dctx(struct zcomp_ctx *ctx)
{
	struct zstd_ctx *zctx = ctx->context;

	if (!zctx)
		return;

	if (zctx->mem)
		vfree(zctx->mem);
	else
		ZSTD_freeDCtx(zctx->ctx);

	kfree(zctx);
}

static int zstd_create_dctx(struct zcomp_params *params, struct zcomp_ctx *ctx)
{
	struct zstd_params *zp = params->drv_data;
	struct zstd_ctx *zctx;

	zctx = kzalloc(sizeof(*zctx), GFP_KERNEL);
	if (!zctx)
		return -ENOMEM;

	ctx->context = zctx;
	if (!zp->ddict) {
		size_t size = ZSTD_DCtxWorkspaceBound();

		zctx->mem = vzalloc(size);
		if (!zctx->mem)
			goto error;
		zctx->ctx = ZSTD_initDCtx(zctx->mem, size);
	} else {
		zctx->ctx = ZSTD_createDCtx_advanced(zp->custom_mem);
	}
	if (!zctx->ctx)
		goto error;

	return 0;

error:
	zstd_destroy_dctx(ctx);
	return -ENOMEM;
}

static int zstd_compress(struct zcomp_params *params, struct zcomp_ctx *ctx,
			 struct zcomp_req *req)
{
	struct zstd_params *zp = params->drv_data;
	struct zstd_ctx *zctx = ctx->context;
	size_t ret;

	if (!zp->cdict)
		ret = ZSTD_compressCCtx(zctx->ctx, req->dst, req->dst_len,
					req->src, req->src_len, zp->cprm);
	else
		ret = ZSTD_compress_usingCDict(zctx->ctx, req->dst, req->dst_len,
					req->src, req->src_len, zp->cdict);
	if (ZSTD_isError(ret))
		return -EINVAL;
	req->dst_len = ret;
	return 0;
}

static int zstd_decompress(struct zcomp_params *params, struct zcomp_ctx *ctx,
			   struct zcomp_req *req)
{
	struct zstd_params *zp = params->drv_data;
	struct zstd_ctx *zctx = ctx->context;
	size_t ret;

	if (!zp->ddict)
		ret = ZSTD_decompressDCtx(zctx->ctx, req->dst, req->dst_len,
					  req->src, req->src_len);
	else
		ret = ZSTD_decompress_usingDDict(zctx->ctx, req->dst, req->dst_len,
					       req->src, req->src_len, zp->ddict);
	if (ret != req->dst_len)
		return -EINVAL;
	return 0;
}

const struct zcomp_ops backend_zstd = {
	.compress	= zstd_compress,
	.decompress	= zstd_decompress,
	.create_cctx	= zstd_create_cctx,
	.destroy_cctx	= zstd_destroy_cctx,
	.create_dctx	= zstd_create_dctx,
	.destroy_dctx	= zstd_destroy_dctx,
	.setup_params	= zstd_setup_params,
	.release_params	= zstd_release_params,
	.name		= "zstd",
};
