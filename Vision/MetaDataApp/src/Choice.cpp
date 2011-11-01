#include "ChoiceFrame.h"
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace boost::filesystem;

void ChoiceFrame::OnOK( wxMouseEvent& event ){

	stringstream s;
	s << dirPicker->GetPath().ToAscii();
	boost::filesystem::path dirPath = s.str().c_str();

	s.str("");
	s << dirPath << "/Images";

	if(!is_directory(s.str().c_str())){
		MessageField->SetLabel( wxT("Unable to Locate \"Images\" directory") );
		return;
	}

	imagePaths.clear();
	LoadImagePaths(s.str().c_str());

	if(imagePaths.size() <= 0){
		MessageField->SetLabel( wxT("No images in image directory") );
		return;
	}

	s.str("");
	s << dirPath << "/Values.xml";

	if (!is_regular_file(s.str().c_str())){
		MessageField->SetLabel( wxT("Unable to open \"Values.xml\"") );
		return;
	}

    frame = new GUIFrame( this, wxID_ANY, wxT("GUI"), wxDefaultPosition, wxSize( 900, 630 ), wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL  );
    frame->SetDirPath(dirPath);
	frame->ChangeImagePaths(imagePaths);

	if(EditExistingXMLradioBtn->GetValue()){
		frame->EditXML(Edit);
	}else if(CreateNewXMLradioBtn->GetValue()){
		frame->EditXML(NewXML);
	}else{
		frame->EditXML(Add);
	}

	boost::property_tree::ptree pt;
	boost::property_tree::read_xml(s.str().c_str(), pt);

	BOOST_FOREACH( boost::property_tree::ptree::value_type& image, pt.get_child("objectTypes") ) {
		stringstream ss;
		ss << image.first;
		frame->AddObject( wxString(ss.str().c_str(), wxConvLocal) );
	}

	BOOST_FOREACH( boost::property_tree::ptree::value_type& image, pt.get_child("backgroundTypes") ) {
		stringstream ss;
		ss << image.first;
		frame->AddBackground( wxString(ss.str().c_str(), wxConvLocal) );
	}

	BOOST_FOREACH( boost::property_tree::ptree::value_type& image, pt.get_child("LightOptions") ) {
		stringstream ss;
		ss << image.first;
		frame->AddLight( wxString(ss.str().c_str(), wxConvLocal) );
	}

	BOOST_FOREACH( boost::property_tree::ptree::value_type& image, pt.get_child("PerspectiveOptions") ) {
		stringstream ss;
		ss << image.first;
		frame->AddPerspective( wxString(ss.str().c_str(), wxConvLocal) );
	}

    frame->Start();
    this->Show(false);
}

void ChoiceFrame::Start(){
    this->Show(true);

}

void ChoiceFrame::LoadImagePaths(const path &itPath){
	directory_iterator LocalImgIt = directory_iterator(itPath);

	while(LocalImgIt != directory_iterator()){
		path p = LocalImgIt->path();

		if(is_directory(p)){
			LoadImagePaths(p);

		}else{
			//If its a image (JPG)
			string extension = p.extension();
			boost::algorithm::to_lower(extension);
			if(extension == ".jpg"){
				bool match = false;
				if(AddToExistingXMLradioBtn->GetValue()){
					boost::property_tree::ptree pt;
					boost::property_tree::read_xml("Test_set/TestSet.xml", pt);

					BOOST_FOREACH( boost::property_tree::ptree::value_type& image, pt.get_child("Test_set") ) {
						path temp = image.second.get("<xmlattr>.path", "");
						if(temp.leaf() == p.leaf()){
							match = true;
							break;
						}
					}
				}
				if(!match){
					imagePaths.push_back(p);
				}
			}
		}
		LocalImgIt++;
	}
}

void ChoiceFrame::OnXMLOption( wxMouseEvent& event ){
	if(event.GetId() == CreateNewXMLradioBtn->GetId()){
		MessageField->SetLabel( wxT("Creates or overrides\nTestSet.xml") );
		CreateNewXMLradioBtn->SetValue(true);
	}else if(event.GetId() == EditExistingXMLradioBtn->GetId()){
		MessageField->SetLabel( wxT("Images already in the xml are\nkept and overwritten if changed") );
		EditExistingXMLradioBtn->SetValue(true);
	}else if(event.GetId() == AddToExistingXMLradioBtn->GetId()){
		MessageField->SetLabel( wxT("Only adds images not available\nin TestSet.xml") );
		AddToExistingXMLradioBtn->SetValue(true);
	}
}

void ChoiceFrame::OnExit( wxMouseEvent& event ){
	this->Close(true);
}
