#TODO remove unused imports.
from qupulse.hardware.awgs.base import AWG, ChannelNotFoundException
from qupulse._program.seqc import HDAWGProgramManager
from qupulse.hardware.util import traced
from typing import Tuple, Callable, Optional
from qupulse.hardware.feature_awg.features import ChannelSynchronization, AmplitudeOffsetHandling, VoltageRange, \
    ProgramManagement, ActivatableChannels, DeviceControl, StatusTable, SCPI, VolatileParameters, \
    ReadProgram, RepetitionMode

from qupulse.utils.types import ChannelID, TimeType, time_from_float
from qupulse._program._loop import Loop, make_compatible
from qupulse._program.seqc import HDAWGProgramManager, UserRegister, WaveformFileSystem
from qupulse.hardware.awgs.base import AWG, ChannelNotFoundException, AWGAmplitudeOffsetHandling
from qupulse.hardware.feature_awg.base import AWGChannelTuple, AWGChannel, AWGDevice, AWGMarkerChannel
from qupulse.pulses.parameters import ConstantParameter
from qupulse.hardware.util import traced


########################################################################################################################
# ChannelTuple
########################################################################################################################
@traced
class HDAWGProgramManagement(ProgramManagement):
    MIN_WAVEFORM_LEN = 192
    WAVEFORM_LEN_QUANTUM = 16
     
    #TODO check if channel_tuple is needed
    def __init__(self, channel_tuple: "HDAWGChannelTuple"):
        pass
     
    def upload(self, name: str,
               program: Loop,
               channels: Tuple[Optional[ChannelID], ...],
               markers: Tuple[Optional[ChannelID], ...],
               voltage_transformation: Tuple[Callable, ...],
               force: bool = False) -> None:
        """Upload a program to the AWG.

        Physically uploads all waveforms required by the program - excluding those already present -
        to the device and sets up playback sequences accordingly.
        This method should be cheap for program already on the device and can therefore be used
        for syncing. Programs that are uploaded should be fast(~1 sec) to arm.

        Args:
            name: A name for the program on the AWG.
            program: The program (a sequence of instructions) to upload.
            channels: Tuple of length num_channels that ChannelIDs of  in the program to use. Position in the list
            corresponds to the AWG channel
            markers: List of channels in the program to use. Position in the List in the list corresponds to
            the AWG channel
            voltage_transformation: transformations applied to the waveforms extracted rom the program. Position
            in the list corresponds to the AWG channel
            force: If a different sequence is already present with the same name, it is
                overwritten if force is set to True. (default = False)

        Known programs are handled in host memory most of the time. Only when uploading the
        device memory is touched at all.

        Returning from setting user register in seqc can take from 50ms to 60 ms. Fluctuates heavily. Not a good way to
        have deterministic behaviour "setUserReg(PROG_SEL, PROG_IDLE);".
        """
        if len(channels) != self.num_channels:
            raise HDAWGValueError('Channel ID not specified')
        if len(markers) != self.num_markers:
            raise HDAWGValueError('Markers not specified')
        if len(voltage_transformation) != self.num_channels:
            raise HDAWGValueError('Wrong number of voltage transformations')

        if name in self.programs and not force:
            raise HDAWGValueError('{} is already known on {}'.format(name, self.identifier))

        # Go to qupulse nanoseconds time base.
        q_sample_rate = self.sample_rate / 10**9

        # Adjust program to fit criteria.
        make_compatible(program,
                        minimal_waveform_length=self.MIN_WAVEFORM_LEN,
                        waveform_quantum=self.WAVEFORM_LEN_QUANTUM,
                        sample_rate=q_sample_rate)

        if self._amplitude_offset_handling == AWGAmplitudeOffsetHandling.IGNORE_OFFSET:
            voltage_offsets = (0.,) * self.num_channels
        elif self._amplitude_offset_handling == AWGAmplitudeOffsetHandling.CONSIDER_OFFSET:
            voltage_offsets = self.offsets()
        else:
            raise ValueError('{} is invalid as AWGAmplitudeOffsetHandling'.format(self._amplitude_offset_handling))

        amplitudes = self.amplitudes()

        if name in self._program_manager.programs:
            self._program_manager.remove(name)

        self._program_manager.add_program(name,
                                          program,
                                          channels=channels,
                                          markers=markers,
                                          voltage_transformations=voltage_transformation,
                                          sample_rate=q_sample_rate,
                                          amplitudes=amplitudes,
                                          offsets=voltage_offsets)

        self._required_seqc_source = self._program_manager.to_seqc_program()
        self._program_manager.waveform_memory.sync_to_file_system(self.master_device.waveform_file_system)

        # start compiling the source (non-blocking)
        self._start_compile_and_upload()

    def remove(self, name: str) -> None:
        """Remove a program from the AWG.

        Also discards all waveforms referenced only by the program identified by name.

        Args:
            name: The name of the program to remove.
        """
        self._program_manager.remove(name)
        self._required_seqc_source = self._program_manager.to_seqc_program()

    def clear(self) -> None:
        """Removes all programs and waveforms from the AWG.

        Caution: This affects all programs and waveforms on the AWG, not only those uploaded using qupulse!
        """
        self._program_manager.clear()
        self._current_program = None
        self._required_seqc_source = self._program_manager.to_seqc_program()
        self._start_compile_and_upload()
        self.arm(None)

    def arm(self, name: Optional[str]) -> None:
        """Load the program 'name' and arm the device for running it. If name is None the awg will "dearm" its current
        program.

        Currently hardware triggering is not implemented. The HDAWGProgramManager needs to emit code that calls
        `waitDigTrigger` to do that.
        """
        if self.num_channels > 8:
            if name is None:
                self._required_seqc_source = ""
            else:
                self._required_seqc_source = self._program_manager.to_seqc_program(name)
            self._start_compile_and_upload()

        if self._required_seqc_source != self._uploaded_seqc_source:
            self._wait_for_compile_and_upload()

        self.user_register(self._program_manager.Constants.TRIGGER_REGISTER, 0)

        if name is None:
            self.user_register(self._program_manager.Constants.PROG_SEL_REGISTER,
                               self._program_manager.Constants.PROG_SEL_NONE)
            self._current_program = None
        else:
            if name not in self.programs:
                raise HDAWGValueError('{} is unknown on {}'.format(name, self.identifier))
            self._current_program = name

            # set the registers of initial repetition counts
            for register, value in self._program_manager.get_register_values(name).items():
                assert register not in (self._program_manager.Constants.PROG_SEL_REGISTER,
                                        self._program_manager.Constants.TRIGGER_REGISTER)
                self.user_register(register, value)

            self.user_register(self._program_manager.Constants.PROG_SEL_REGISTER,
                               self._program_manager.name_to_index(name) | int(self._program_manager.Constants.NO_RESET_MASK, 2))

        # this was a workaround for problems in the past and I totally forgot why it was here
        # for ch_pair in self.master.channel_tuples:
        #    ch_pair._wait_for_compile_and_upload()
        self.enable(True)
 
# Features
class HDAWGChannelTuple(AWGChannelTuple):
    pass

class HDAWGException(Exception):
    """Base exception class for HDAWG errors."""
    pass

class HDAWGValueError(HDAWGException, ValueError):
    pass
