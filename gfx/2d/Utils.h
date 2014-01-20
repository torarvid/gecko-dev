/* -*- Mode: c++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#ifndef MOZILLA_GFX_UTILS_H_
#define MOZILLA_GFX_UTILS_H_

#include "2D.h"

namespace mozilla {
namespace gfx {

void
PremultiplySurface(DataSourceSurface* srcSurface,
                   DataSourceSurface* destSurface = nullptr);

}
}

#endif /* MOZILLA_GFX_UTILS_H_ */
