#include <cstdlib>
#include <wx/wx.h>
#include <huniplacer_gui/huniplacer_frame_impl.h>

using namespace huniplacer_gui;

huniplacer_frame_impl* frame;

class app : public wxApp
{
	bool OnInit();
};

IMPLEMENT_APP(app)

bool app::OnInit()
{
	frame = new huniplacer_frame_impl();
	frame->Show(true);
	SetTopWindow(frame);
	return true;
}
