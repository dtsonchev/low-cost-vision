#include <MetaDataApp/ChoiceFrame.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost::filesystem;

void ChoiceFrame::OnOK(wxMouseEvent& event) {

	stringstream s;
	s << dirPicker->GetPath().ToAscii();
	boost::filesystem::path dirPath = s.str().c_str();

	s.str("");
	s << dirPath << "/Images";

	if (!is_directory(s.str().c_str())) {
		MessageField->SetLabel(wxT("Unable to Locate \"Images\" directory"));
		return;
	}

	stringstream xmlpath;
	xmlpath << XMLPicker->GetPath().ToAscii();
	boost::filesystem::path xmlPath = xmlpath.str().c_str();

	imagePaths.clear();
	LoadImagePaths(s.str().c_str(), xmlpath.str().c_str());

	if (imagePaths.size() <= 0) {
		MessageField->SetLabel(wxT("No images in image directory"));
		return;
	}

	s.str("");
	s << dirPicker->GetPath().ToAscii() << "/Values.xml";

	if (!is_regular_file(s.str().c_str())) {
		MessageField->SetLabel(wxT("Unable to open the Values.xml file"));
		return;
	}

	frame = new GUIFrame(this, wxID_ANY, wxT("Give for each object the values"),
			wxDefaultPosition, wxSize(500, 780),
			wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);
	frame->SetDirPath(dirPath);
	frame->SetXMLPath(xmlPath);
	frame->ChangeImagePaths(imagePaths);

	if (EditExistingXMLradioBtn->GetValue()) {
		frame->EditXML(Edit);
	} else if (CreateNewXMLradioBtn->GetValue()) {
		frame->EditXML(NewXML);
	} else {
		frame->EditXML(Add);
	}

	boost::property_tree::ptree pt;
	boost::property_tree::read_xml(s.str().c_str(), pt);

	BOOST_FOREACH( boost::property_tree::ptree::value_type& image, pt.get_child("values.objectTypes") )
			{
				string objectType = image.second.get("<xmlattr>.name", "");

				if (objectType != "") {
					frame->AddObject(wxString(objectType.c_str(), wxConvLocal));
				}
			}

	BOOST_FOREACH( boost::property_tree::ptree::value_type& image, pt.get_child("values.backgroundTypes") )
			{
				string backgroundType = image.second.get("<xmlattr>.name", "");

				if (backgroundType != "") {
					frame->AddBackground(
							wxString(backgroundType.c_str(), wxConvLocal));
				}
			}

	BOOST_FOREACH( boost::property_tree::ptree::value_type& image, pt.get_child("values.LightOptions") )
			{
				string LightOption = image.second.get("<xmlattr>.name", "");

				if (LightOption!= "") {
					frame->AddLight(wxString(LightOption.c_str(), wxConvLocal));
				}
			}

	BOOST_FOREACH( boost::property_tree::ptree::value_type& image, pt.get_child("values.PerspectiveOptions") )
			{
				string PerspectiveOption = image.second.get("<xmlattr>.name",
						"");

				if (PerspectiveOption != "") {
					frame->AddPerspective(
							wxString(PerspectiveOption.c_str(), wxConvLocal));
				}
			}

	frame->Start();
	frame->SetPosition( wxPoint (0, 0));
	EditExistingXMLradioBtn->SetValue(true);
	this->Show(false);
}

void ChoiceFrame::Start() {
	EditExistingXMLradioBtn->SetValue(true);
	this->Show(true);
}

void ChoiceFrame::LoadImagePaths(const path &itPath, const path &xmlPath) {
	directory_iterator LocalImgIt = directory_iterator(itPath);

	while (LocalImgIt != directory_iterator()) {
		path p = LocalImgIt->path();

		if (is_directory(p)) {
			LoadImagePaths(p, xmlPath);

		} else {
			//If its a image (JPG)
			string extension = p.extension();
			boost::algorithm::to_lower(extension);
			if (extension == ".jpg") {
				bool match = false;
				if (AddToExistingXMLradioBtn->GetValue()) {
					boost::property_tree::ptree pt;
					stringstream xmlpath;
					xmlpath << xmlPath;
					boost::property_tree::read_xml(xmlpath.str().c_str(), pt);

					BOOST_FOREACH( boost::property_tree::ptree::value_type& image, pt.get_child("Test_set") )
							{
								path temp = image.second.get("<xmlattr>.path",
										"");
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

void ChoiceFrame::OnXMLOption(wxMouseEvent& event) {
	if (event.GetId() == CreateNewXMLradioBtn->GetId()) {
		MessageField->SetLabel(wxT("Override an existing .xml file"));
		CreateNewXMLradioBtn->SetValue(true);
	} else if (event.GetId() == EditExistingXMLradioBtn->GetId()) {
		MessageField->SetLabel(wxT("If an image within this directory\n"
				"is edited this image is overwritten\n"
				"within the xml"));
		EditExistingXMLradioBtn->SetValue(true);
	} else if (event.GetId() == AddToExistingXMLradioBtn->GetId()) {
		MessageField->SetLabel(
				wxT("Only adds images not available\nin TestSet.xml"));
		AddToExistingXMLradioBtn->SetValue(true);
	}
}

void ChoiceFrame::OnExit(wxMouseEvent& event) {
	this->Close(true);
}
