/*
 * cli_commands.h
 *
 *  Created on: Jul 1, 2016
 *      Author: luktor99
 */

#ifndef CLI_COMMANDS_H_
#define CLI_COMMANDS_H_
#include <FreeRTOS_CLI.h>

static const char * const CLI_welcome_message = "\f\033[1;36mWelcome to the BlueRay Micromouse CLI!\r\nType \"help\" to view a list of available commands.\r\n\n\033[0m";
static const char * const CLI_prompt = "\033[1;34mcmd>\033[0m";

// Registers all functions
void register_commands(void);

#endif /* CLI_COMMANDS_H_ */
