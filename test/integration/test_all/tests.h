#pragma once

// Each *_tests.cpp file in this directory keeps its individual test
// functions internal (static) and exposes a single grouping function that
// runs them all. all.test.cpp drives the suite by calling these in order
// between UNITY_BEGIN() and UNITY_END().

void test_smoke(void);       // smoke_tests.cpp
void test_wifi(void);        // wifi_tests.cpp
void test_api_setup(void);   // api_setup_tests.cpp
void test_api_display(void); // api_display_tests.cpp
void test_tls_resume(void);  // tls_resume_tests.cpp
