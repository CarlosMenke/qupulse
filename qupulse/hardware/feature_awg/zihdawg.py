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
from qupulse.pulses.parameters import ConstantParameter
from qupulse.hardware.util import traced


@traced
class HDAWGProgramManagement(ProgramManagement):
    #TODO add __init__ function
     
     
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
 
class HDAWGException(Exception):
    """Base exception class for HDAWG errors."""
    pass

class HDAWGValueError(HDAWGException, ValueError):
    pass


