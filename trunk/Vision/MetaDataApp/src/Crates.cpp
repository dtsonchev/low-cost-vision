/*
 * Crates.cpp
 *
 *  Created on: Nov 1, 2011
 *      Author: glenn
 */

#include "CrateGUI.h"
#include <wx/dc.h>
#include <wx/dcclient.h>
#include <sstream>

void CrateGUI::OnLeftDown( wxMouseEvent& event ){
	leftMouseDownInImage = true;

	if(LT_Radio->GetValue() || RT_Radio->GetValue() || LB_Radio->GetValue()){
		LTX = event.GetX();
		LTY = event.GetY();
	}else if(LTLTCorner_Radio->GetValue()){
		RBX = positionValues[(0*4)+0] + positionValues[(0*4)+2];
		RBY = positionValues[(0*4)+1] + positionValues[(0*4)+3];
	}else if(LTRLCorner_Radio->GetValue()){
		LTX = positionValues[(0*4)+0];
		LTY = positionValues[(0*4)+1];
	}else if(RTLTCorner_Radio->GetValue()){
		RBX = positionValues[(1*4)+0] + positionValues[(1*4)+2];
		RBY = positionValues[(1*4)+1] + positionValues[(1*4)+3];
	}else if(RTRLCorner_Radio->GetValue()){
		LTX = positionValues[(1*4)+0];
		LTY = positionValues[(1*4)+1];
	}else if(LBLTCorner_Radio->GetValue()){
		RBX = positionValues[(2*4)+0] + positionValues[(2*4)+2];
		RBY = positionValues[(2*4)+1] + positionValues[(2*4)+3];
	}else if(LBRLCorner_Radio->GetValue()){
		LTX = positionValues[(2*4)+0];
		LTY = positionValues[(2*4)+1];
	}
}

void CrateGUI::OnLeftUp( wxMouseEvent& event ){
	std::stringstream s;
	leftMouseDownInImage = false;

	if(LT_Radio->GetValue() || LTRLCorner_Radio->GetValue() || LTLTCorner_Radio->GetValue()){

		if(LT_Radio->GetValue()){
			RT_Radio->SetValue(true);
		}

		positionValues[(0*4)+0] = LTX > RBX ? RBX : LTX;
		positionValues[(0*4)+1] = LTY > RBY ? RBY : LTY;
		positionValues[(0*4)+2] = abs(RBX - LTX);
		positionValues[(0*4)+3] = abs(RBY - LTY);

		s.str("");
		s << "(" << positionValues[(0*4)+0] << ", " << positionValues[(0*4)+1] << ")";
		LTLT_TxtField->SetLabel( wxString(s.str().c_str(), wxConvLocal));
		s.str("");
		s << "(" << positionValues[(0*4)+0] + positionValues[(0*4)+2] << ", " <<
				positionValues[(0*4)+1] + positionValues[(0*4)+3] << ")";
		LTRB_TxtField->SetLabel( wxString(s.str().c_str(), wxConvLocal));

	}else if(RT_Radio->GetValue() || RTRLCorner_Radio->GetValue() || RTLTCorner_Radio->GetValue()){

		if(RT_Radio->GetValue()){
			LB_Radio->SetValue(true);
		}

		positionValues[(1*4)+0] = LTX > RBX ? RBX : LTX;
		positionValues[(1*4)+1] = LTY > RBY ? RBY : LTY;
		positionValues[(1*4)+2] = abs(RBX - LTX);
		positionValues[(1*4)+3] = abs(RBY - LTY);

		s.str("");
		s << "(" << positionValues[(1*4)+0] << ", " << positionValues[(1*4)+1] << ")";
		RTLT_TxtField->SetLabel( wxString(s.str().c_str(), wxConvLocal));
		s.str("");
		s << "(" << positionValues[(1*4)+0] + positionValues[(1*4)+2] << ", " <<
				positionValues[(1*4)+1] + positionValues[(1*4)+3] << ")";
		RTRB_TxtField->SetLabel( wxString(s.str().c_str(), wxConvLocal));

	}else if(LB_Radio->GetValue() || LBRLCorner_Radio->GetValue() || LBLTCorner_Radio->GetValue()){

		if(LB_Radio->GetValue()){
			LT_Radio->SetValue(true);
		}

		positionValues[(2*4)+0] = LTX > RBX ? RBX : LTX;
		positionValues[(2*4)+1] = LTY > RBY ? RBY : LTY;
		positionValues[(2*4)+2] = abs(RBX - LTX);
		positionValues[(2*4)+3] = abs(RBY - LTY);

		s.str("");
		s << "(" << positionValues[(2*4)+0] << ", " << positionValues[(2*4)+1] << ")";
		LBLT_TxtField->SetLabel( wxString(s.str().c_str(), wxConvLocal));
		s.str("");
		s << "(" << positionValues[(2*4)+0] + positionValues[(2*4)+2] << ", " <<
				positionValues[(2*4)+1] + positionValues[(2*4)+3] << ")";
		LBRB_TxtField->SetLabel( wxString(s.str().c_str(), wxConvLocal));
	}
	DrawBoxesOnImage(-1);
}

void CrateGUI::OnLeftMotion( wxMouseEvent& event ){
	if(leftMouseDownInImage){

		if(LTLTCorner_Radio->GetValue() || RTLTCorner_Radio->GetValue() || LBLTCorner_Radio->GetValue()){
			LTX = event.GetX();
			LTY = event.GetY();
		}
		if(LT_Radio->GetValue() || RT_Radio->GetValue() || LB_Radio->GetValue() ||
			LTRLCorner_Radio->GetValue() || RTRLCorner_Radio->GetValue() || LBRLCorner_Radio->GetValue()){
			RBX = event.GetX();
			RBY = event.GetY();
		}

		if(LT_Radio->GetValue() || LTLTCorner_Radio->GetValue() || LTRLCorner_Radio->GetValue()){
			DrawBoxesOnImage(0);
		}else if(RT_Radio->GetValue() || RTLTCorner_Radio->GetValue() || RTRLCorner_Radio->GetValue()){
			DrawBoxesOnImage(1);
		}else if(LB_Radio->GetValue() || LBLTCorner_Radio->GetValue() || LBRLCorner_Radio->GetValue()){
			DrawBoxesOnImage(2);
		}
	}
}

void CrateGUI::OnDonePressed( wxMouseEvent& event ){

	for(int marknr = 0; marknr < 3; marknr++){
		for(int valuenr = 0; valuenr < 4; valuenr++){
			if(positionValues[(marknr*4)+valuenr] == 0){

				if(marknr == 0){
					LT_Radio->SetFocus();
				}else if(marknr == 1){
					RT_Radio->SetFocus();
				}else if(marknr == 2){
					LB_Radio->SetFocus();
				}

				return;
			}

			if(valuenr == 0 || valuenr == 2){
				positionValues[(marknr*4)+valuenr] = positionValues[(marknr*4)+valuenr] * widthScale;
			}else{
				positionValues[(marknr*4)+valuenr] = positionValues[(marknr*4)+valuenr] * heightScale;
			}
		}
	}

	if(QRCode_TxtField->GetValue() == wxT("QR code")){
		QRCode_TxtField->SetFocus();
		return;
	}

	barcode = QRCode_TxtField->GetValue();

	this->GetParent()->Show(true);
	this->Show(false);
}

void CrateGUI::ClearPositionValues(){
	LTX = LTY = RBX = RBY = 0;

	for(int marknr = 0; marknr < 3; marknr++){
		for(int valuenr = 0; valuenr < 4; valuenr++){
			positionValues[(marknr*4)+valuenr] = 0;
		}
	}

	LTLT_TxtField->SetLabel(wxString( ("(000,000)"), wxConvLocal));
	LTRB_TxtField->SetLabel(wxString( ("(000,000)"), wxConvLocal));
	RTLT_TxtField->SetLabel(wxString( ("(000,000)"), wxConvLocal));
	RTRB_TxtField->SetLabel(wxString( ("(000,000)"), wxConvLocal));
	LBLT_TxtField->SetLabel(wxString( ("(000,000)"), wxConvLocal));
	LBRB_TxtField->SetLabel(wxString( ("(000,000)"), wxConvLocal));
}

void CrateGUI::Start(wxImage cutImage, int x, int y, wxString imagePath){

	ClearPositionValues();

	LT_Radio->SetValue(true);
	LT_Radio->SetFocus();

	originalX = x;
	originalY = y;

	QRCode_TxtField->SetLabel(wxT("QR code"));
	pathField->SetLabel( imagePath );

	leftMouseDownInImage = false;
	image = cutImage;
	widthScale = (double)cutImage.GetWidth() / (double)imageWidth;
	heightScale = (double)cutImage.GetHeight() / (double)imageHeight;
	image.Rescale(imageWidth, imageHeight, 1);

	UpdateImageField();

	this->Show(true);
	this->GetParent()->Show(false);
}

bool CrateGUI::getValues(int *values, wxString &barcode){

	for(int marknr = 0; marknr < 3; marknr++){
		for(int valuenr = 0; valuenr < 4; valuenr++){
			if(positionValues[(marknr*4)+valuenr] == 0){
				return false;
			}
			values[(marknr*4)+valuenr] = positionValues[(marknr*4)+valuenr];
			if(valuenr == 0){
				values[(marknr*4)+0] += originalX;
			}else if(valuenr == 1){
				values[(marknr*4)+1] += originalY;
			}
		}
	}

	if(QRCode_TxtField->GetValue() != wxT("QR code")){
		barcode = this->barcode;
		return true;
	}

	return false;
}

void CrateGUI::UpdateImageField(){
	imageField->SetBitmap(image);
	imageField->SetSize(imageWidth, imageHeight);
}

void CrateGUI::DrawBoxesOnImage(int skipNr){
	UpdateImageField();
	imageField->Update();

	wxPaintDC dc(imageField);
	for(int i = 0; i < 3; i++){
		if(i != skipNr){
			if(positionValues[(i*4)+0] > 0 && positionValues[(i*4)+1] > 0 &&
					positionValues[(i*4)+2] > 0 && positionValues[(i*4)+3] > 0){

				int TopX = positionValues[(i*4)+0] + 3;
				int TopY = positionValues[(i*4)+1] + 3;
				int BottomX = positionValues[(i*4)+2] + TopX;
				int BottomY = positionValues[(i*4)+3] + TopY;

				dc.DrawLine(TopX, TopY, BottomX, TopY);//Top horizontal
				dc.DrawLine(TopX, TopY, TopX, BottomY);//Left vertical

				dc.DrawLine(BottomX, TopY, BottomX, BottomY);//Right vertical
				dc.DrawLine(TopX, BottomY, BottomX, BottomY);//Bottom horizontal

				dc.DrawRectangle( wxPoint((TopX + (positionValues[(i*4)+2]/2)) -1 , (TopY + (positionValues[(i*4)+3]/2)) -1), wxSize( 3, 3) );
			}
		}else{
			dc.DrawLine(LTX + 3, LTY + 3, RBX + 3, LTY + 3);//Top horizontal
			dc.DrawLine(LTX + 3, LTY + 3, LTX + 3, RBY + 3);//Left vertical

			dc.DrawLine(RBX + 3, LTY + 3, RBX + 3, RBY + 3);//Right vertical
			dc.DrawLine(LTX + 3, RBY + 3, RBX + 3, RBY + 3);//Bottom horizontal

			//dc.DrawRectangle( wxPoint((TLX + ((BRX- TLX)/2)) -1 , (TLY + ((BRY -TLY)/2)) -1), wxSize( 3, 3) );
		}
	}
}

void CrateGUI::OnQRFocus( wxFocusEvent& event ){
	QRCode_TxtField->SetValue(wxT(""));
}
