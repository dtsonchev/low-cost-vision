#ifndef __ConverterGUI__
#define __ConverterGUI__

#include <wx/string.h>
#include <wx/stattext.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/statbmp.h>
#include <wx/combobox.h>
#include <wx/radiobut.h>
#include <wx/statline.h>
#include <wx/checkbox.h>
#include <wx/filepicker.h>
#include <wx/frame.h>
#include <wx/dc.h>
#include <wx/dcclient.h>

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

/*
using namespace std;
using namespace boost::filesystem;
*/

typedef enum XMLOption{
	Add,
	Edit,
	NewXML
}XMLOption;

/**
 * @brief This class is made for an GUI
 * @details With this GUI we can create meta data for images in a selected directory by doing the options within the GUI
 * @author Glenn
 * @version 2.0
 * @date 10-2011
 */
class GUIFrame : public wxFrame 
{

	private:
		///@brief a constant for the width of the image
		static const int imageWidth = 640;
		///@brief a constant for the heigth of the image
		static const int imageHeight = 480;
		///@brief this becomes our xml with all the nodes
		boost::property_tree::ptree pt;

		///@brief The top left X coordinate for the box for surrounding the object
		int x;
		///@brief The top left Y coordinate for the box for surrounding the object
		int y;
		///@brief The width for the box for surrounding the object
		int width;
		///@brief The height for the box for surrounding the object
		int height;
		///@brief The top X coordinate for the center line for the object
		int TopX;
		///@brief The top Y coordinate for the center line for the object
		int TopY;
		///@brief The bottom X coordinate for the center line for the object
		int BottomX;
		///@brief The bottom Y coordinate for the center line for the object
		int BottomY;
		///@brief An images in a xml file can be added, edit and a new one can be created
		int editXML;
		///@brief A variable for the current object number
		unsigned int currentObjectNr;
		///@brief The total amount of objects in the current image specified by the user
		unsigned long int AmountOfObjects;
		///@brief A variable for the rotation for the current object
		double rotation;
		///@brief A variable for the height scale at which the image is resized
		double heightScale;
		///@brief A variable for the width scale at which the image is resized
		double widthScale;
		///@brief A variable for keeping track of the left mouse button is pressed
		bool mousePressedInImage;
		///@brief A variable for the name of the current object
		char* objectName;
		///@brief is an variable for which is set if the image is available in the xml
		bool AlreadyInXML;

		///@brief A variable for the current image
		wxImage image;
		///@brief A variable for stepping through all the images
		boost::filesystem::directory_iterator imgIt;
		///@brief A variable for the path which you selected
		boost::filesystem::path dirPath;
		///@brief A vector variable for each image path in dirPath/Images
		std::vector<boost::filesystem::path> imagePaths;
		///@brief A variable for stepping through all the images
		std::vector<boost::filesystem::path>::iterator imagePathsIt;

		/**
		 * @fn UpdateImageField()
		 * @brief Updates the image in the image field with the image viraible
		 */
		void UpdateImageField();
		/**
		 * @fn DrawOnImageAccordingToRadioButtonOption(int eventX, int eventY)
		 * @brief Draws a box or line depending on the radio button option (Surround box and Center line)
		 * @param eventX the x from the click event
		 * @param eventY the y from the click event
		 */
		void DrawOnImageAccordingToRadioButtonOption(int eventX, int eventY);
		/**
		 * @fn DrawBoxAndRotationLineOnImage()
		 * @brief Draws a box from x, y, width, height. \n and a line for the rotation with TopX, TopY, BottomX and BottomY
		 */
		void DrawBoxAndRotationLineOnImage();
		/**
		 * @fn ChangeImagePath(const wxString& path)
		 * @brief Changes the image path
		 * @param objectName The path were the image is located
		 */
		void ChangeImagePath(const wxString& path);
		/**
		 * @fn GetPathFromChosenDir(boost::filesystem::path &path)
		 * @brief this function takes the image path and subtracts the path of dirPath \n
		 * and returns this in path
		 */
		void GetPathFromChosenDir(boost::filesystem::path &path);
		/**
		 * @fn GetRotation()
		 * @brief Gets the rotation from TopX, TopY, BottomX and BottomY in degrees
		 */
		double GetRotation();

	protected:
		///@brief A pointer to the window which started this one
		//wxWindow* parentWindow;

		wxStaticText* BackgroundTextField;
		wxFilePickerCtrl* filePicker;
		wxStaticBitmap* ImageField;
		wxDirPickerCtrl* dirPicker;
		wxStaticText* AmountOfObjectsLabel;
		wxTextCtrl* AmountOfObjectsTxtField;
		wxComboBox* ObjectComboBox;
		wxComboBox* BackgroundComboBox;
		wxComboBox* LightingComboBox;
		wxComboBox* PerspectiveComboBox;
		wxRadioButton* SurroundBox_radioBtn;
		wxRadioButton* LUC_RadioBtn;
		wxStaticText* LUC_Label;
		wxRadioButton* RLC_RadioBtn;
		wxStaticText* RLC_Label;
		wxCheckBox* NoRotationCheckBox;
		wxRadioButton* CenterLine_radioBtn;
		wxRadioButton* CenterTop_RadioBtn;
		wxStaticText* CenterTop_Label;
		wxRadioButton* CenterBottom_RadioBtn;
		wxStaticText* CenterBottom_Label;
		wxButton* NextObjectButton;
		wxButton* SkipButton;
		wxStaticText* MessageLabel;

		/**
		 * @fn OnNextObjectButton(wxCommandEvent& event)
		 * @brief The function which is called when the next button is pressed
		 * @details when this button is pressed the inputed values are written to an xml
		 * also is either an new image loaded are values cleared.
		 * @param event the event created when the next button is pressed
		 */
		virtual void OnNextObjectButton(wxCommandEvent& event);
		/**
		 * @fn OnAmountOfObjects(wxFocusEvent& event)
		 * @brief The function which is called when the focus on AmountOfObjectsTxtField is lost
		 * @details when there are more than one objects within the image the next buttons
		 * label changes into next image or into next object
		 * @param event the event created the focus is lost
		 */
		virtual void OnAmountOfObjects(wxFocusEvent& event);
		/**
		 * @fn OnLeftMouseRelease(wxMouseEvent& event)
		 * @brief The function which is called when the left mouse left mouse button is released within the image field
		 * @details only when the mouse is released the values inputed by the user are updated
		 * @param event the event created when the left mouse button is released within the image field
		 */
		virtual void OnLeftMouseRelease(wxMouseEvent& event);
		/**
		 * @fn OnLeftMousePressed(wxMouseEvent& event)
		 * @brief The function which is called when the left mouse button is pressed within the image field
		 * @details while the left mouse button is pressed the a rectangle or line is drawn real time onto the image
		 * @param event the event created when the left mouse button is pressed within the image field
		 */
		virtual void OnLeftMousePressed(wxMouseEvent& event);
		/**
		 * @fn OnImageMotion(wxMouseEvent& event)
		 * @brief The function which is called when the left mouse button is pressed and moved within the image field
		 * @details draws the first rectangle or line
		 * @param event the event created when the left mouse button is pressed and moved within the image field
		 */
		virtual void OnImageMotion(wxMouseEvent& event);
		/**
		 * @fn NoRotation(wxMouseEvent& event)
		 * @brief The function which is called when the left mouse button is pressed on the box
		 * @details makes the center line option (in)visible
		 * @param event the event created when the left mouse button is pressed on the box
		 */
		virtual void NoRotation(wxMouseEvent& event);
		/**
		 * @fn OnSkip( wxMouseEvent& event )
		 * @brief The function which is called when the skip button is pressed
		 * @details calls NextImage()
		 * @param event the event created when the skip button is pressed
		 */
		virtual void OnSkip( wxMouseEvent& event );
	
	public:
		///@brief Constructor for the GUIFrame
		GUIFrame( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("GUI"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 600,590 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		///@brief Destructor for the GUIFrame
		~GUIFrame();

		/**
		 * @fn Start()
		 * @brief call this function to start going through all the images in the image directory
		 * @details sets all the variables, and makes same GUI object invisible
		 */
		void Start();
		/**
		 * @fn AddBackground(wxString objectName)
		 * @brief Adds a background type to the drop down box in the GUI
		 * @param backgroundName The name of the object you want to add to the combo box list
		 */
		void AddBackground(const wxString &backgroundName);
		/**
		 * @fn AddLight(wxString objectName)
		 * @brief Adds a light type to the drop down box in the GUI
		 * @param lightName The name of the object you want to add to the combo box list
		 */
		void AddLight(const wxString &lightName);
		/**
		 * @fn AddPerspective(wxString objectName)
		 * @brief Adds a perspective type to the drop down box in the GUI
		 * @param perspectiveName The name of the object you want to add to the combo box list
		 */
		void AddPerspective(const wxString &perspectiveName);
		/**
		 * @fn AddObject(wxString objectName)
		 * @brief Adds an object type to the drop down box in the GUI
		 * @param objectName The name of the object you want to add to the combo box list
		 */
		void AddObject(const wxString &objectName);
		/**
		 * @fn ChangeImagePaths(vstd::vector<boost::filesystem::path> &imagePaths)
		 * @brief changes the vector with image paths
		 * @param imagePaths A vector variable for each image path in dirPath/Images
		 */
		void ChangeImagePaths(const std::vector<boost::filesystem::path> &imagePaths);
		/**
		 * @fn EditXML(int edit)
		 * @brief sets the variable editXML
		 * @param edit the value at which editXML is set
		 */
		void EditXML(int edit);
		/**
		 * @fn SetDirPath(boost::filesystem::path path)
		 * @brief sets the directory path
		 * @param path the value at which dirPath is set
		 */
		void SetDirPath(boost::filesystem::path path);
		/**
		 * @fn NextImage()
		 * @brief Selects the next image in the image directory and calls the ChangeImage function
		 */
		void NextImage();
};

#endif //__ConverterGUI__
