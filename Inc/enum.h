#ifndef __ENUM_H
#define __ENUM_H

enum Moving_State
{
	At_Buttom,
	Moving,
	Finished,
	At_Top
};

enum Circle_State
{
	Circle_Init,
	Circle_Moving,
	Circle_Idle
};

enum CoreXY_Command
{
	CoreXY_Home,
	CoreXY_TogXY,
};

enum CoreXY_Action
{
	Up,
	Down,
	Stop
};
#endif
