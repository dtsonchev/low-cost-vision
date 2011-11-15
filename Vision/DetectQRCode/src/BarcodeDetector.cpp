#include <boost/filesystem.hpp>
#include "BarcodeDetector.h"
#include "MagickMat.h"
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
