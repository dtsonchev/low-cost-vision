//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CrateGUI
// File:           ChoiceFrameImpl.cpp
// Description:    The implementation of the functions in the GUI
// Author:         Glenn Meerstra
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
#include "CrateGUI/ChoiceFrameImpl.h"
#include "CrateGUI/ChoiceFrame.h"
#include "CrateGUI/CrateAppImpl.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/exceptions.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <sstream>

#include <stdexcept>

#ifdef __CDT_PARSER__
	#define foreach(a, b) for(a : b)
#else
	#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

using namespace boost::filesystem;

void ChoiceFrameImpl::OnXMLOption(wxMouseEvent& event) {
	if (event.GetId() == CreateNewXMLradioBtn->GetId()) {
		MessageField->SetLabel(wxT("Crates a .xml file"));
		CreateNewXMLradioBtn->SetValue(true);
	} else if (event.GetId() == EditExistingXMLradioBtn->GetId()) {
		MessageField->SetLabel(wxT("If an image within this directory\n"
				"is edited this image is overwritten\n"
				"within the xml"));
		EditExistingXMLradioBtn->SetValue(true);
	} else if (event.GetId() == AddToExistingXMLradioBtn->GetId()) {
		MessageField->SetLabel(
				wxT("Only adds images not available\nin the .xml file"));
		AddToExistingXMLradioBtn->SetValue(true);
	}
}

void ChoiceFrameImpl::OnExit(wxMouseEvent& event) {
	this->Close(true);
}

void ChoiceFrameImpl::OnOK(wxMouseEvent& event) {
	boost::filesystem::path dirPath = std::string(
			dirPicker->GetPath().ToAscii()).c_str();
	std::stringstream s;
	s << dirPath << "/Metadata.xml";
	path xmlPath = s.str();//std::string(XMLPicker->GetPath().ToAscii());

	if (!CreateNewXMLradioBtn->GetValue()) {
		if (xmlPath == "") {
			MessageField->SetLabel(wxT("No .xml file selected"));
			return;
		}

		if (!is_regular_file(xmlPath)) {
			MessageField->SetLabel(wxT("The .xml file is not a regular file"));
			return;
		}

	} else {
		MessageField->SetLabel(
				wxT("create new .xml file in\nselected directory"));
	}

	s.str("");
	s << dirPath << "/Images";

	if (!is_directory(s.str().c_str())) {
		MessageField->SetLabel(wxT("Unable to Locate\n\"Images\" directory"));
		return;
	}

	imagePaths.clear();
	LoadImagePaths(s.str().c_str(), xmlPath.string().c_str());

	if (imagePaths.size() <= 0) {
		MessageField->SetLabel(wxT("No (new) images in image \ndirectory"));
		return;
	}

	s.str("");
	s << dirPath << "/Values.xml";

	if (!is_regular_file(s.str().c_str())) {
		MessageField->SetLabel(wxT("Unable to open the Values.xml file"));
		return;
	}

	crateApp = new CrateAppImpl(this);

	crateApp->SetDirPath(dirPath);
	crateApp->SetXMLPath(xmlPath);
	crateApp->ChangeImagePaths(imagePaths);

	if (EditExistingXMLradioBtn->GetValue()) {
		crateApp->EditXML(crateApp->Edit);
	} else if (CreateNewXMLradioBtn->GetValue()) {
		crateApp->EditXML(crateApp->NewXML);
	} else {
		crateApp->EditXML(crateApp->Add);
	}

	boost::property_tree::ptree pt;
	boost::property_tree::read_xml(s.str().c_str(), pt);
	try{

		foreach( boost::property_tree::ptree::value_type& image, pt.get_child("values.backgroundTypes") ){
			std::string backgroundType = image.second.get("<xmlattr>.name", "");

			if (backgroundType != "") {
				crateApp->AddBackground(
						wxString(backgroundType.c_str(), wxConvLocal));
			}
		}

		foreach( boost::property_tree::ptree::value_type& image, pt.get_child("values.lightOptions") ){
			std::string LightOption = image.second.get("<xmlattr>.name", "");

			if (LightOption!= "") {
				crateApp->AddLight(wxString(LightOption.c_str(), wxConvLocal));
			}
		}

		foreach( boost::property_tree::ptree::value_type& image, pt.get_child("values.perspectiveOptions") ){
			std::string PerspectiveOption = image.second.get("<xmlattr>.name", "");

			if (PerspectiveOption != "") {
				crateApp->AddPerspective(
						wxString(PerspectiveOption.c_str(), wxConvLocal));
			}
		}

		AddToExistingXMLradioBtn->SetValue(true);
		crateApp->Start();
		crateApp->Show(true);
		this->Show(false);
	}catch(boost::property_tree::ptree_bad_path& ex){
		MessageField->SetLabel(wxT("Values.xml contains the false values"));
	}
}

void ChoiceFrameImpl::LoadImagePaths(const path &itPath, const path &xmlPath) {
	directory_iterator LocalImgIt = directory_iterator(itPath);

	while (LocalImgIt != directory_iterator()) {
		path p = LocalImgIt->path();

		if (is_directory(p)) {
			LoadImagePaths(p, xmlPath);

		} else {
			//If its a image (JPG)
			std::string extension = p.extension();
			boost::algorithm::to_lower(extension);
			if (extension == ".jpg" || extension == ".png") {
				bool match = false;
				if (AddToExistingXMLradioBtn->GetValue()) {
					boost::property_tree::ptree pt;
					boost::property_tree::read_xml(xmlPath.string(), pt);

					foreach( boost::property_tree::ptree::value_type& imageObject, pt.get_child("metadata") ){
						path temp = imageObject.second.get("<xmlattr>.path", "");
						if (temp.leaf() == p.leaf()) {
							match = true;
							break;
						}
					}
				}
				if (!match) {
					imagePaths.push_back(p);
				}
			}
		}
		LocalImgIt++;
	}
}

ChoiceFrameImpl::ChoiceFrameImpl(wxWindow* parent, wxWindowID id,
		const wxString& title, const wxPoint& pos, const wxSize& size,
		long style) :
		ChoiceFrame(parent, id, title, pos, size, style), crateApp(NULL) {
}
