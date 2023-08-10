# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from clearpath_config.common.types.list import ListConfig
from clearpath_config.common.utils.dictionary import merge_dict
from clearpath_config.platform.types.attachment import BaseAttachment
from clearpath_config.platform.types.bumper import Bumper
from clearpath_config.platform.types.fender import Fender
from clearpath_config.platform.types.structure import Structure
from clearpath_config.platform.types.top_plate import TopPlate
from typing import List


class AttachmentListConfig(ListConfig[BaseAttachment, str]):
    def __init__(self) -> None:
        super().__init__(
            uid=lambda obj: obj.get_name(),
            obj_type=BaseAttachment,
            uid_type=str
        )

    def to_dict(self) -> dict:
        d = {}
        for decoration in self.get_all():
            merge_dict(d, decoration.to_dict())
        return d


# Base Attachments Config
# - to be used by all other configurations.
class BaseAttachmentsConfig:

    def __init__(self) -> None:
        # Standard Platform Attachments
        self.__bumpers = ListConfig[Bumper, str](
            uid=ListConfig.uid_name,
            obj_type=Bumper,
            uid_type=str)
        self.__top_plates = ListConfig[TopPlate, str](
            uid=ListConfig.uid_name,
            obj_type=TopPlate,
            uid_type=str)
        self.__structures = ListConfig[Structure, str](
            uid=ListConfig.uid_name,
            obj_type=Structure,
            uid_type=str)
        self.__fenders = ListConfig[Fender, str](
            uid=ListConfig.uid_name,
            obj_type=Fender,
            uid_type=str)

    def to_dict(self):
        d = {}
        for attachment in self.get_all():
            merge_dict(d, attachment)
        return d

    @property
    def top_plates(self):
        return self.__top_plates

    @top_plates.setter
    def top_plates(self, value: List[TopPlate] | ListConfig) -> None:
        if isinstance(value, list):
            self.__top_plates.set_all(value)
        elif isinstance(value, ListConfig):
            self.__top_plates = value
        else:
            assert isinstance(value, list) or isinstance(value, ListConfig), (
                "Top plates must be list of 'TopPlate' or 'ListConfig'"
            )

    @property
    def bumpers(self):
        return self.__bumpers

    @bumpers.setter
    def bumpers(self, value: List[Bumper] | ListConfig) -> None:
        if isinstance(value, list):
            self.__bumpers.set_all(value)
        elif isinstance(value, ListConfig):
            self.__bumpers = value
        else:
            assert isinstance(value, list) or isinstance(value, ListConfig), (
                "Bumpers must be list of 'Bumper' or 'ListConfig'"
            )

    @property
    def structures(self):
        return self.__structures

    @structures.setter
    def structures(self, value: List[Structure] | ListConfig) -> None:
        if isinstance(value, list):
            self.__structures.set_all(value)
        elif isinstance(value, ListConfig):
            self.__structures = value
        else:
            assert isinstance(value, list) or isinstance(value, ListConfig), (
                "Structures must be list of 'Structure' or 'ListConfig'"
            )

    @property
    def fenders(self):
        return self.__fenders

    @fenders.setter
    def fenders(self, value: List[Fender] | ListConfig) -> None:
        if isinstance(value, list):
            self.__fenders.set_all(value)
        elif isinstance(value, ListConfig):
            self.__fenders = value
        else:
            assert isinstance(value, list) or isinstance(value, ListConfig), (
                "Fenders must be list of 'Fender' or 'ListConfig'"
            )

    def get_all(self) -> List[BaseAttachment]:
        attachments = []
        attachments.extend(self.bumpers.get_all())
        attachments.extend(self.top_plates.get_all())
        attachments.extend(self.structures.get_all())
        attachments.extend(self.fenders.get_all())
        return attachments
