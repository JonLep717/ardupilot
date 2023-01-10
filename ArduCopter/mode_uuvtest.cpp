#include "Copter.h"

/*
 * Init and run calls for test mode
 */

// stand-by_run - everything pass-through
// should be called at 100hz or more
void ModeTest::run()
{
	motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
}
