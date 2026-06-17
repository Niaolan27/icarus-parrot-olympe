#  Copyright (c) 2023 Parrot Drones SAS
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of the Parrot Company nor the names
#    of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written
#    permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
#  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
#  SUCH DAMAGE.

# fsup mandatory library
from fsup.genmission import AbstractMission
from msghub_utils import msg_id

###################################################
# Stages and transitions

from fsup.missions.default.ground.stage import (
    GROUND_STAGE as DEF_GROUND_STAGE,
)
from fsup.missions.default.takeoff.stage import (
    TAKEOFF_STAGE as DEF_TAKEOFF_STAGE,
)
from fsup.missions.default.hovering.stage import (
    HOVERING_STAGE as DEF_HOVERING_STAGE,
)
from fsup.missions.default.flying.stage import (
    FLYING_STAGE as DEF_FLYING_STAGE,
)
from fsup.missions.default.landing.stage import (
    LANDING_STAGE as DEF_LANDING_STAGE,
)
from fsup.missions.default.critical.stage import (
    CRITICAL_STAGE as DEF_CRITICAL_STAGE,
)

from fsup.missions.default.mission import TRANSITIONS as DEF_TRANSITIONS

###################################################
# Messages

# AirSDK Service messages (cv_road)
import road_runner.cv_road.messages_pb2 as rr_service_msgs

# AirSDK/Olympe-facing mission messages
import parrot.missions.samples.yolo.airsdk.messages_pb2 as yolo_msgs

###################################################
# Messages channel

_CV_ROAD_SERVICE_CHANNEL = "unix:/tmp/road-runner-cv-road-service"

###################################################
# Mission


class Mission(AbstractMission):
    def __init__(self, env):
        super().__init__(env)

        # Olympe-facing mission messages
        self.ext_ui_msgs = None

        # AIRSDK SERVICE (cv_road) <---> FSUP
        self.airsdk_service_cv_road_messages_channel = None
        self.airsdk_service_cv_road_handler_messages = None
        self.airsdk_service_cv_road_messages_observer = None

    def on_load(self):
        # Olympe-facing mission messages
        self.ext_ui_msgs = self.env.make_airsdk_service_pair(yolo_msgs)

        # AIRSDK SERVICE (cv_road) <---> FSUP
        self.airsdk_service_cv_road_messages_channel = (
            self.mc.start_client_channel(
                _CV_ROAD_SERVICE_CHANNEL
            )
        )

    def on_unload(self):
        # AIRSDK SERVICE (cv_road) <---> FSUP
        self.mc.stop_channel(self.airsdk_service_cv_road_messages_channel)
        self.airsdk_service_cv_road_messages_channel = None

        # Olympe-facing mission messages
        self.ext_ui_msgs = None

    def on_activate(self):
        # Olympe-facing mission messages
        self.ext_ui_msgs.attach(self.env.airsdk_channel, True)

        self.airsdk_service_cv_road_handler_messages = (
            self.mc.attach_client_service_pair(
                self.airsdk_service_cv_road_messages_channel,
                rr_service_msgs,
                forward_events=True,
            )
        )

        self.airsdk_service_cv_road_messages_observer = (
            self.airsdk_service_cv_road_handler_messages.evt.observe(
                {
                    msg_id(
                        rr_service_msgs.Event,
                        "yolo_detection",
                    ): self._relay_yolo_detection,
                }
            )
        )

        self._send_cv_road_enable(True)

    def on_deactivate(self):
        # AIRSDK SERVICE (cv_road)
        self._send_cv_road_enable(False)

        self.airsdk_service_cv_road_messages_observer.unobserve()
        self.airsdk_service_cv_road_messages_observer = None

        self.mc.detach_client_service_pair(self.airsdk_service_cv_road_handler_messages)  # noqa: E501
        self.airsdk_service_cv_road_handler_messages = None

        # Olympe-facing mission messages
        self.ext_ui_msgs.detach()

    def states(self):
        return [
            DEF_GROUND_STAGE,
            DEF_TAKEOFF_STAGE,
            DEF_HOVERING_STAGE,
            DEF_FLYING_STAGE,
            DEF_LANDING_STAGE,
            DEF_CRITICAL_STAGE,
        ]

    def transitions(self):
        return DEF_TRANSITIONS

    def _send_cv_road_enable(self, enable):
        self.airsdk_service_cv_road_handler_messages.cmd.sender.enable_cv(enable)  # noqa: E501
        self.log.info(f"cv_road enable {enable}")

    def _relay_yolo_detection(self, *args):
        msg = args[-1]
        yolo_detection = msg.yolo_detection

        self.log.info("Relaying yolo_detection event: %s", yolo_detection)

        self.ext_ui_msgs.evt.sender.yolo_detection(
            class_id=yolo_detection.class_id,
            confidence=yolo_detection.confidence,
            x=yolo_detection.x,
            y=yolo_detection.y,
            width=yolo_detection.width,
            height=yolo_detection.height,
        )
