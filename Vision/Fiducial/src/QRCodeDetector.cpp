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
// License:        GNU GPL v3
//
// This file is part of DetectQRCode.
//
// DetectQRCode is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DetectQRCode is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DetectQRCode.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include <sstream>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include "QRCodeDetector.h"

QRCodeDetector::QRCodeDetector() {
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
}

QRCodeDetector::~QRCodeDetector() {}

bool QRCodeDetector::detect(cv::Mat& image, std::string &result) {
	try {
		zbar::Image zbarImage(image.cols, image.rows, "Y800", (void*)image.data, image.cols * image.rows);

		int amountOfScannedResults = scanner.scan(zbarImage);

		if (amountOfScannedResults) {
			zbar::Image::SymbolIterator symbol = zbarImage.symbol_begin();

			if(symbol->get_type() != zbar::ZBAR_QRCODE) return false;

			result = symbol->get_data();

		} else {
			return false;
		}
	} catch (std::exception &e) {
		return false;
	}
	return true;
}

void QRCodeDetector::detectCrates(cv::Mat& image, std::vector<Crate> &crates) {
	try {
		zbar::Image zbarImage(image.cols, image.rows, "Y800", (void*)image.data, image.cols * image.rows);

		int amountOfScannedResults = scanner.scan(zbarImage);

		if (amountOfScannedResults > 0) {
			zbar::Image::SymbolIterator it = zbarImage.symbol_begin();
			for(; it!=zbarImage.symbol_end(); ++it) {
				if(it->get_type() == zbar::ZBAR_QRCODE) {
					std::vector<cv::Point2f> points;
					points.push_back(cv::Point2f(it->get_location_x(1), it->get_location_y(1)));
					points.push_back(cv::Point2f(it->get_location_x(0), it->get_location_y(0)));
					points.push_back(cv::Point2f(it->get_location_x(3), it->get_location_y(3)));

					//std::cout << "Before: " << points << std::endl;

					// Refine to subpixel-percision
					// TODO: Utilize more corners to improve robustness and percision
					cv::TermCriteria term;
					term.epsilon = 0.001;
					term.type = cv::TermCriteria::EPS;
					cv::cornerSubPix(image, points, cv::Size(5,5), cv::Size(-1,-1), term);

					//std::cout << "After: " << points << std::endl;

					crates.push_back(Crate(it->get_data(), points));
				}
			}
		}
	} catch (std::exception &e) {
		return;
	}
}
