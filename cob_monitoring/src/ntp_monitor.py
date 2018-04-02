#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
<<<<<<< HEAD
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This file has been copied from https://github.com/PR2/pr2_computer_monitor in order to support this feature for indigo indepenendly from PR2 dependencies
=======
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function
>>>>>>> ff88c55e7d57e20b993d43ea12638352daddfec7

import sys
import socket
from subprocess import Popen, PIPE
import time
import re

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class NtpMonitor():
    def __init__(self, argv=sys.argv):
        rospy.init_node("ntp_monitor")
        self.parse_args(argv)

        stat = DiagnosticStatus()
        stat.level = 0
        stat.name = '%s NTP Offset' % self.diag_hostname
        stat.message = "OK"
        stat.hardware_id = self.diag_hostname
        stat.values = []
        self.msg = DiagnosticArray()
        self.msg.header.stamp = rospy.get_rostime()
        self.msg.status = [stat]

        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=1)
        self.diag_timer = rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)
        self.monitor_timer = rospy.Timer(rospy.Duration(60.0), self.update_diagnostics)

    def update_diagnostics(self, event):
        stat = DiagnosticStatus()
        stat.level = 0
        stat.name = '%s NTP Offset' % self.diag_hostname
        stat.message = "OK"
        stat.hardware_id = self.diag_hostname
        stat.values = []

        for st,host,off in [(stat, self.ntp_server, self.offset)]:
            try:
                p = Popen(["ntpdate", "-q", host], stdout=PIPE, stdin=PIPE, stderr=PIPE)
                res = p.wait()
                (o,e) = p.communicate()
            except OSError, (errno, msg):
                if errno == 4:
                    break #ctrl-c interrupt
                else:
                    raise
            if (res == 0):
                measured_offset = float(re.search("offset (.*),", o).group(1))*1000000

                st.level = DiagnosticStatus.OK
                st.message = "OK"
                st.values = [ KeyValue("NTP Server" , self.ntp_server),
                              KeyValue("Offset (us)", str(measured_offset)),
                              KeyValue("Offset tolerance (us)", str(off)),
                              KeyValue("Offset tolerance (us) for Error", str(self.error_offset)) ]

                if (abs(measured_offset) > off):
                    st.level = DiagnosticStatus.WARN
                    st.message = "NTP Offset Too High"
                if (abs(measured_offset) > self.error_offset):
                    st.level = DiagnosticStatus.ERROR
                    st.message = "NTP Offset Too High"

            else:
                st.level = DiagnosticStatus.ERROR
                st.message = "Error Running ntpdate. Returned %d" % res
                st.values = [ KeyValue("NTP Server" , self.ntp_server),
                              KeyValue("Offset (us)", "N/A"),
                              KeyValue("Offset tolerance (us)", str(off)),
                              KeyValue("Offset tolerance (us) for Error", str(self.error_offset)),
                              KeyValue("Output", o),
                              KeyValue("Errors", e) ]

        self.msg = DiagnosticArray()
        self.msg.header.stamp = rospy.get_rostime()
        self.msg.status = [stat]

    def publish_diagnostics(self, event):
        self.diag_pub.publish(self.msg)

    def parse_args(self, argv=sys.argv):
        import optparse
        parser = optparse.OptionParser(usage="usage: ntp_monitor ntp-hostname []")
        parser.add_option("--offset", dest="offset",
                          action="store", default=500,
                          help="Offset from NTP host", metavar="OFFSET")
        parser.add_option("--error-offset", dest="error_offset",
                          action="store", default=5000000,
                          help="Offset from NTP host. Above this is error", metavar="OFFSET")
        parser.add_option("--diag-hostname", dest="diag_hostname",
                          help="Computer name in diagnostics output (ex: 'c1')",
                          metavar="DIAG_HOSTNAME",
                          action="store", default=None)
        options, args = parser.parse_args(rospy.myargv())

        if (len(args) != 2):
            parser.error("Invalid arguments. Must have HOSTNAME [args]. %s" % args)
            print('Invalid arguments.', file=sys.stderr)
            sys.exit(0)

        try:
            offset = int(options.offset)
            error_offset = int(options.error_offset)
        except:
            parser.error("Offsets must be numbers")
            print('Offsets must be numbers', file=sys.stderr)
            sys.exit(0)

        self.ntp_server = args[1]
        self.diag_hostname = options.diag_hostname
        hostname = socket.gethostname()
        if self.diag_hostname is None:
            self.diag_hostname = hostname

        self.offset = rospy.get_param('~offset', offset)
        self.error_offset = rospy.get_param('~error_offset', error_offset)


if __name__ == "__main__":
    ntp = NtpMonitor(argv=sys.argv)
    rospy.spin()
