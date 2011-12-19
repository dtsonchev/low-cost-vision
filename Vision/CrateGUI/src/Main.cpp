#include <wx/wx.h>
#include <CrateGUI/ChoiceFrameImpl.h>
#include <CrateGUI/CrateAppImpl.h>

/**
 * @mainpage Meta data GUI
 * @section This program indexes images
 * This program indexes images located in a 'Images' directory next to
 * this directory there needs to be a .txt file which contains:
 * \n -backgroundTypes \n Black \n White \n etc. \n
 * \n -LightOptions \n TL light \n No extra light \n etc. \n
 * \n -PerspectiveOptions \n 2D \n 3D \n etc.
 */

///@brief this class is made to launch the GUI
class MyApp: public wxApp {
	/**
	 * @brief this function launches the GUI
	 */
	virtual bool OnInit();
};

IMPLEMENT_APP(MyApp)

bool MyApp::OnInit() {
	ChoiceFrameImpl *startFrame;
	startFrame = new ChoiceFrameImpl( NULL );
	startFrame->Show(true);

	return true;
}
