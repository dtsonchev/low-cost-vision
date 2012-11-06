//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        DetectQRCode
// File:           BarcodeDetector.cpp
// Description:    Detects barcodes and extract values
// Author:         Glenn Meerstra & Zep Mouris
// Notes:          ...
//
// License: newBSD 
//  
// Copyright © 2012, HU University of Applied Sciences Utrecht. 
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************

#include <boost/filesystem.hpp>
#include "DetectQRCode/BarcodeDetector.h"
#include "DetectQRCode/MagickMat.h"
//#include "MagickBitmapSource.h"
#include <sstream>

/*#include <zxing/common/Counted.h>
#include <zxing/Binarizer.h>
#include <zxing/MultiFormatReader.h>
#include <zxing/Result.h>
#include <zxing/ReaderException.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/common/HybridBinarizer.h>
#include <exception>
#include <zxing/Exception.h>
#include <zxing/common/IllegalArgumentException.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>

#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/multi/qrcode/QRCodeMultiReader.h>
#include <zxing/multi/ByQuadrantReader.h>
#include <zxing/multi/MultipleBarcodeReader.h>
#include <zxing/multi/GenericMultipleBarcodeReader.h>

using namespace Magick;
using namespace std;
using namespace zxing;
using namespace zxing::multi;
using namespace zxing::qrcode;*/

DetectBarcode::DetectBarcode(){
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

bool DetectBarcode::detect(cv::Mat image, std::string &result){
	Magick::Image magickImage;
	MagickMatConverter converter;

	if(converter.Mat2Magick(image, magickImage)){
		/*try{
			zxing::Ref<Binarizer> binarizer(NULL);
			zxing::Ref<Magick::MagickBitmapSource> source(new Magick::MagickBitmapSource(magickImage));

			Magick::DecodeHints hints(Magick::DecodeHints::DEFAULT_HINT);
			hints.setTryHarder(true);

			binarizer = new zxing::HybridBinarizer(source);
			zxing::Ref<zxing::BinaryBitmap> binary(new zxing::BinaryBitmap(binarizer));
			zxing::Ref<zxing::Result> codeResult(zxing::decode(binary, hints));
			result = codeResult->getText()->getText();
		}catch(std::exception &e){
			try{
				binarizer = new zxing::GlobalHistogramBinarizer(source);
				zxing::Ref<zxing::BinaryBitmap> binary(new zxing::BinaryBitmap(binarizer));
				zxing::Ref<zxing::Result> codeResult(decode(binary, hints));
				result = codeResult->getText()->getText();

			}catch(std::exception &e){
			*/
				try{
					int width = magickImage.columns();
					int height = magickImage.rows();
					Magick::Blob blob;
					magickImage.modifyImage();
					magickImage.write(&blob, "GRAY", 8);

					const void *raw = blob.data();
					zbar::Image zbarImage(width, height, "Y800", raw, width * height);

					int amountOfScannedResults = scanner.scan(zbarImage);

					if(amountOfScannedResults){
						zbar::Image::SymbolIterator symbol = zbarImage.symbol_begin();
						result += symbol->get_data();
					}else{
						return false;
					}
					scanner.recycle_image(zbarImage);
					zbarImage.set_data(NULL, 0);
				}catch(std::exception &e){
					return false;
				}
			/*}
		}*/
	}
	return true;
}

DetectBarcode::~DetectBarcode(){}
