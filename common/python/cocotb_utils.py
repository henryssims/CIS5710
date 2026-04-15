"""This file has code used across several testbenches."""

from pathlib import Path
import os, re

# Use half the available cores for Verilator's parallel build
os.environ['MAKEFLAGS'] = '-j%d' % int(os.cpu_count()/2)

VERILATOR_FLAGS = [
    '--assert',
    '-Wall',
    '-Wno-DECLFILENAME',
    '--trace',
    '--trace-fst',
    '--trace-structs',
    # NB: --trace-max-array must be ≥ size of the memory (in 4B words) for memory to appear in the waveforms
    '--trace-max-array',str(2**18)
    ]

# directory where our simulator will compile our tests + code
SIM_BUILD_DIR = "sim_build"

# simulator to use
SIM = "verilator"

# NB: these paths are relative to the testbench file, not cocotb_utils.py
RISCV_TESTS_PATH = Path('../../riscv-tests/isa')
RISCV_BENCHMARKS_PATH = Path('../../riscv-tests/benchmarks')

POINTS_FILE = 'points.json'

def assertEquals(expected, actual, msg=''):
    """Wrapper around regular assert, with automatic formatting of values in hex"""
    if expected != actual:
        if isinstance(expected,int):
            assert_msg = f'expected 0x{int(expected):X} but was 0x{int(actual):X}'
        else:
            assert_msg = f'expected {expected} but was {actual}'
        if msg != '':
            assert_msg += f': {msg}'
            pass
        assert expected == actual, assert_msg
        pass
    pass

def shouldGenerateWaveforms():
    """Returns True if simulator should generate waveforms, and False otherwise. Disable waveforms (only) when running autograder, for faster execution."""
    insideAutograder = os.path.isdir('/autograder/submission')
    if insideAutograder:
        print('[cocotb_utils.py] autograder run detected, waveform generation disabled')
        return False
    return True

def aggregateTestResults(*results):
    """Aggregates total/failed counts from all arguments, where each argument is a call to cocotb.runner.get_results()"""
    total_tests = sum([r[0] for r in results])
    total_failed_tests = sum([r[1] for r in results])
    return { 
        'tests_total': total_tests,
        'tests_failed': total_failed_tests,
        'tests_passed': total_tests - total_failed_tests
        }

def extractSVEnum(file_path, enum_name):
    """Parse a SystemVerilog enum into a dictionary mapping int to string values"""
    with open(file_path, 'r') as file:
        content = file.read()

        pattern = re.compile(rf'typedef\s+enum\s+\{{(.*?)\}}\s+{enum_name};', re.DOTALL)
        match = pattern.search(content)
        
        if not match:
            raise ValueError(f"Enum {enum_name} not found in the file.")

        enum_body = match.group(1)
        enum_constants = re.findall(r'(\w+)\s*=\s*(\d+)', enum_body)
        
        return {name: int(value) for name, value in enum_constants}
    pass

def intToEnumString(value, enum_dict):
    result = []
    for name, enum_value in enum_dict.items():
        if value & enum_value:
            result.append(name)
            pass
        pass
    return ','.join(result)

def enumStringToInt(enum_string, enum_dict):
    value = 0
    for name in enum_string.split(','):
        if name in enum_dict:
            value |= enum_dict[name]
        else:
            raise ValueError(f"Invalid enum name: {name}")
        pass
    return value

_CYCLE_STATUS_ENUM = None

def _trace_wb_pc(dut):
    if hasattr(dut.datapath, 'trace_writeback_pc'):
        return dut.datapath.trace_writeback_pc.value.integer
    return dut.datapath.trace_completed_pc.value.integer

def _trace_wb_insn(dut):
    if hasattr(dut.datapath, 'trace_writeback_insn'):
        return dut.datapath.trace_writeback_insn.value.integer
    return dut.datapath.trace_completed_insn.value.integer

def _trace_wb_status(dut):
    if hasattr(dut.datapath, 'trace_writeback_cycle_status'):
        return dut.datapath.trace_writeback_cycle_status.value.integer
    return dut.datapath.trace_completed_cycle_status.value.integer

def handleTrace(dut, trace, traceIdx, tracingMode):
    global _CYCLE_STATUS_ENUM
    if _CYCLE_STATUS_ENUM is None:
        _CYCLE_STATUS_ENUM = extractSVEnum('../../hw3-singlecycle/cycle_status.sv', 'cycle_status_e')
        pass
    if tracingMode == 'generate':
        traceElem = {}
        traceElem['cycle'] = dut.datapath.cycles_current.value.integer
        traceElem['trace_writeback_pc'] = f'0x{_trace_wb_pc(dut):x}'
        traceElem['trace_writeback_insn'] = f'0x{_trace_wb_insn(dut):08x}'
        status_int = _trace_wb_status(dut)
        traceElem['trace_writeback_cycle_status'] = intToEnumString(status_int, _CYCLE_STATUS_ENUM)
        trace.append(traceElem)
    elif tracingMode == 'compare':
        traceElem = trace[traceIdx]
        msg = f'trace validation error at cycle {traceElem["cycle"]}'
        expected_pc = traceElem.get('trace_writeback_pc', traceElem.get('trace_completed_pc'))
        expected_insn = traceElem.get('trace_writeback_insn', traceElem.get('trace_completed_insn'))
        expected_status = traceElem.get('trace_writeback_cycle_status', traceElem.get('trace_completed_cycle_status'))
        assert expected_pc is not None, f'{msg}: missing expected PC trace key'
        assert expected_insn is not None, f'{msg}: missing expected insn trace key'
        assert expected_status is not None, f'{msg}: missing expected cycle status trace key'
        assertEquals(int(expected_pc,16), _trace_wb_pc(dut), msg)
        assertEquals(int(expected_insn,16), _trace_wb_insn(dut), msg)
        actual_status = intToEnumString(_trace_wb_status(dut), _CYCLE_STATUS_ENUM)
        assertEquals(expected_status, actual_status, msg)
        pass
    return
