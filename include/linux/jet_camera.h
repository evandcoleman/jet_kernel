#ifndef _JET_CAMERA_H
#define _JET_CAMERA_H

struct cam_plat_data {
	long reset_gpio; 
	struct clk *ext_clk;
};
#endif