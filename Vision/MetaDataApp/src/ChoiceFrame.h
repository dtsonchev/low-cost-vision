#ifndef __CHOICEFRAME__
#define __CHOICEFRAME__

#include "ConverterGUI.h"

#include <wx/string.h>
#include <wx/filepicker.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/stattext.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/button.h>
#include <wx/frame.h>
#include <wx/filedlg.h>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

/**
 * @class ChoiceFrame ChoiceFrame.h "src/ChoiceFrame"
 * @brief this class makes an frame at which 3 choices can be made and  a directory needs to be chosen
 * \n the choices are make a new xml file,
 * \n edit an old xml or add only new images
 * \n the directory needs to contain an directory 'Image', and a document 'Values.txt'
 */
class ChoiceFrame: public wxFrame {

private:
	bool Load;
	GUIFrame *frame;
	std::vector<boost::filesystem::path> imagePaths;

protected:
	wxDirPickerCtrl* dirPicker;
	wxFilePickerCtrl* XMLPicker;
	wxStaticText* MessageField;
	wxButton* ExitButton;
	wxButton* OKButton;
	wxRadioButton* CreateNewXMLradioBtn;
	wxRadioButton* EditExistingXMLradioBtn;
	wxRadioButton* AddToExistingXMLradioBtn;

	/**
	 * @fn OnOK( wxMouseEvent& event )
	 * @brief The function which is called when the OK button is pressed
	 * @details when this button is pressed the inputed the comboboxen from GUIFrame are filled
	 * and all the image paths are collected
	 * @param event the event created when the OK button is pressed
	 */
	virtual void OnOK(wxMouseEvent& event);
	/**
	 * @fn OnXMLOption( wxMouseEvent& event )
	 * @brief The function which is called when one of the radio buttons is pressed
	 * @param event the event created when the OK button is pressed
	 */
	virtual void OnXMLOption(wxMouseEvent& event);
	/**
	 * @fn OnExit( wxMouseEvent& event )
	 * @brief The function which is called when the Exit button is pressed
	 * @details this frame and the GUIFrame are closed
	 * @param event the event created when the Exit button is pressed
	 */
	virtual void OnExit(wxMouseEvent& event);
	/**
	 * @fn LoadImagePaths(boost::filesystem::path itPath)
	 * @brief Loads all the image paths from the image directory
	 * @param itPath the root map from where the images are supposed to be loaded
	 */
	void LoadImagePaths(const boost::filesystem::path &itPath,
			const boost::filesystem::path &xmlPath);

public:
	///@brief Constructor for ChoiceFrame
	ChoiceFrame(wxWindow* parent, wxWindowID id = wxID_ANY,
			const wxString& title = wxEmptyString, const wxPoint& pos =
					wxDefaultPosition, const wxSize& size = wxSize(-1, -1),
			long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);
	~ChoiceFrame();

	/**
	 * @fn Start()
	 * @brief this function creates an GUIFrame and shows the ChoiceFrame
	 */
	void Start();
};

#endif //__CHOICEFRAME__
