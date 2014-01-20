/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "BasicCanvasLayer.h"
#include "basic/BasicLayers.h"          // for BasicLayerManager
#include "mozilla/mozalloc.h"           // for operator new
#include "nsAutoPtr.h"                  // for nsRefPtr
#include "nsCOMPtr.h"                   // for already_AddRefed
#include "nsISupportsImpl.h"            // for Layer::AddRef, etc
#include "gfx2DGlue.h"

class gfxContext;

using namespace mozilla::gfx;
using namespace mozilla::gl;

namespace mozilla {
namespace layers {

// Code to convert Layer to SourceSurface for masking:
/*

AutoMaskData maskData;
if (!GetMaskData(aMaskLayer, &maskData)) {
  NS_WARNING("Invalid mask data");
  return;
}
gfxASurface *thebesSurface = maskData.GetSurface();
SourceSurface* mask =
  gfxPlatform::GetPlatform()->GetSourceSurfaceForSurface(aTarget,
                                                         thebesSurface);
*/
void
BasicCanvasLayer::Paint(DrawTarget* aTarget, SourceSurface* aMaskSurface)
{
  if (IsHidden())
    return;

  FirePreTransactionCallback();
  UpdateSurface();
  FireDidTransactionCallback();

  CompositionOp mixBlendMode = GetEffectiveMixBlendMode();
  PaintWithOpacity(aTarget,
                   GetEffectiveOpacity(),
                   aMaskSurface,
                   mixBlendMode != CompositionOp::OP_OVER ? mixBlendMode : GetOperator());
}

void
BasicCanvasLayer::DeprecatedPaint(gfxContext* aContext, Layer* aMaskLayer)
{
  if (IsHidden())
    return;

  FirePreTransactionCallback();
  DeprecatedUpdateSurface();
  FireDidTransactionCallback();

  gfxContext::GraphicsOperator mixBlendMode = DeprecatedGetEffectiveMixBlendMode();
  DeprecatedPaintWithOpacity(aContext,
                             GetEffectiveOpacity(),
                             aMaskLayer,
                             mixBlendMode != gfxContext::OPERATOR_OVER ? mixBlendMode : DeprecatedGetOperator());
}

already_AddRefed<CanvasLayer>
BasicLayerManager::CreateCanvasLayer()
{
  NS_ASSERTION(InConstruction(), "Only allowed in construction phase");
  nsRefPtr<CanvasLayer> layer = new BasicCanvasLayer(this);
  return layer.forget();
}

}
}
