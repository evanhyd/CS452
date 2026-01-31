#pragma once

extern "C" {
int RegisterAs(const char* name);
int WhoIs(const char* name);
}

namespace name_server {
int createNameServerTask(int priority);
void testTask();
} // namespace name_server
