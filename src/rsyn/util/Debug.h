/* Copyright 2014-2018 Rsyn
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RSYN_ASSERT_H
#define RSYN_ASSERT_H

#include <iostream>

#define rsynAbort(msg) \
	std::cout << "\n"; \
	std::cout << "ASSERTION: " << msg << "\n"; \
	std::cout << __FILE__  << ":" << __LINE__ << "\n"; \
	std::cout << std::flush; \
	std::exit(1);

#define rsynAssert(condition, msg) \
	if (!(condition)) { \
		std::cout << "\n"; \
		std::cout << "ASSERTION: " << #condition << "\n"; \
		std::cout << __FILE__  << ":" << __LINE__ << "\n"; \
		std::cout << msg << "\n"; \
		std::cout << std::flush; \
		std::exit(1); \
	} // end if

#endif

