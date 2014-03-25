/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/. */

#include "WebrtcExtVideoCodec.h"
#include "ExtVideoCodec.h"

namespace mozilla {

VideoEncoder* ExtVideoCodec::CreateEncoder() {
  WebrtcExtVideoEncoder *enc =
      new WebrtcExtVideoEncoder();
  return enc;
}

VideoDecoder* ExtVideoCodec::CreateDecoder() {
  WebrtcExtVideoDecoder *dec =
      new WebrtcExtVideoDecoder();
  return dec;
}

}
