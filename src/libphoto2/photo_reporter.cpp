/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <cstdio>

#include "gphoto2_ros/photo_reporter.hpp"

void photo_reporter::contextError( GPContext *context, const char *error_string, void *data )
{
  (void)context;
  (void)data;

  std::string str_to_test = "PTP No Device";
  if(std::string(error_string).find(str_to_test) != std::string::npos ) {
      std::cout << "Device lost" << std::endl;
      photo_reporter::is_connected_ = false;
  }
  else {
      std::cerr << "\nphoto_reporter: Context error \n" << error_string << std::endl;
  }
}

void photo_reporter::contextStatus( GPContext *context, const char *status_string, void *data )
{
  (void)context;
  (void)data;
  std::cout << "photo_reporter: Status " << status_string << std::endl;
}

void photo_reporter::error( const std::string& function_name )
{
  std::cerr << "photo_reporter: Error executing function '" << function_name << "'." << std::endl;
}

void photo_reporter::error( const std::string& function_name,
                            const std::string& additional_message )
{
  error( function_name );
  std::cerr << additional_message << std::endl;
}


