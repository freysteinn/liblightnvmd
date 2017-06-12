/*
 * lba - logical block addressing (LBA) functions
 *
 * Copyright (C) 2015-2017 Javier Gonzáles <javier@cnexlabs.com>
 * Copyright (C) 2015-2017 Matias Bjørling <matias@cnexlabs.com>
 * Copyright (C) 2015-2017 Simon A. F. Lund <slund@cnexlabs.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define _XOPEN_SOURCE 700
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <liblightnvm.h>
#include <nvm_dev.h>

ssize_t nvm_lba_pwrite(struct nvm_dev *dev, const void *buf, size_t count,
		       off_t offset)
{
	if (count < 1) {
		errno = EINVAL;
		return -1;
	}
	if (offset < 0) {
		errno = EINVAL;
		return -1;
	}
	if (count % dev->geo.sector_nbytes) {
		errno = EINVAL;
		return -1;
	}
	if (offset % dev->geo.sector_nbytes) {
		errno = EINVAL;
		return -1;
	}

	return pwrite(dev->fd, buf, count, offset);
}

ssize_t nvm_lba_pread(struct nvm_dev *dev, void *buf, size_t count,
		      off_t offset)
{
	if (count < 1) {
		errno = EINVAL;
		return -1;
	}
	if (offset < 0) {
		errno = EINVAL;
		return -1;
	}
	if (count % dev->geo.sector_nbytes) {
		errno = EINVAL;
		return -1;
	}
	if (offset % dev->geo.sector_nbytes) {
		errno = EINVAL;
		return -1;
	}

	return pread(dev->fd, buf, count, offset);
}

