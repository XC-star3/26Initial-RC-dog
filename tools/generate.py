#!/usr/bin/env python3
"""Generate the RC-dog LibXR platform and XRobot application entrypoints."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
LIBXR_ROOT = ROOT / "Middlewares/Third_Party/LibXR"
CONFIG_PATH = ROOT / "User/libxr_config.yaml"
XROBOT_CONFIG_PATH = ROOT / "User/xrobot.yaml"
IOC_PATH = ROOT / "dm02.ioc"
MANIFEST_PATH = ROOT / "User/codegen_manifest.cmake"
REQUIREMENTS_PATH = ROOT / "tools/requirements-codegen.txt"

GENERATED_PATHS = (
    ROOT / "User/app_main.cpp",
    ROOT / "User/app_main.h",
    ROOT / "User/xrobot_main.hpp",
)

MINIMUM_VERSIONS = {"libxr": "5.2.4", "xrobot": "0.3.1"}
REQUIRED_TOOLS = {
    "xr_parse_ioc": ("--directory", "--output"),
    "xr_gen_code_stm32": ("--input", "--output", "--xrobot", "--libxr-config"),
    "xrobot_gen_main": ("--config", "--output"),
}

REQUIRED_LIBXR_HEADERS = {
    "Middlewares/Third_Party/LibXR/driver/st/stm32_usb_dev.hpp": (
        "class STM32USBDeviceOtgHS",
        "struct EPInConfig",
    ),
    "Middlewares/Third_Party/LibXR/src/driver/usb/device/cdc/cdc_uart.hpp": (
        "class CDCUart",
        "Endpoint::EPNumber data_in_ep_num",
    ),
    "Middlewares/Third_Party/LibXR/src/middleware/app_framework/hardware.hpp": (
        "class HardwareContainer",
        "struct Entry",
    ),
    "Middlewares/Third_Party/LibXR/system/freertos/libxr_system.hpp": (
        "void PlatformInit(uint32_t timer_pri",
        "uint32_t timer_stack_depth",
    ),
}


class CodegenError(RuntimeError):
    """Raised when the installed generators or project inputs are incompatible."""


@dataclass(frozen=True)
class Toolchain:
    libxr_version: str
    xrobot_version: str
    xr_parse_ioc: str
    xr_gen_code_stm32: str
    xrobot_gen_main: str


@dataclass(frozen=True)
class PlatformConfig:
    software_timer_priority: int
    software_timer_stack: int
    fdcan_queue_size: int
    sbus_rx_size: int
    sbus_queue_size: int
    sbus_dma_section: str
    spi_rx_size: int
    spi_tx_size: int
    spi_dma_threshold: int
    usb_rx_fifo_size: int
    usb_ep0_buffer_size: int
    usb_ep1_out_buffer_size: int
    usb_ep1_in_buffer_size: int
    usb_ep2_in_buffer_size: int
    usb_ep0_in_fifo_size: int
    usb_ep1_in_fifo_size: int
    usb_ep2_in_fifo_size: int
    usb_cdc_rx_size: int
    usb_cdc_tx_size: int
    usb_cdc_queue_size: int
    usb_vid: int
    usb_pid: int
    usb_bcd: int
    usb_manufacturer: str
    usb_product: str
    usb_serial: str
    fdcan_front_alias: str
    fdcan_rear_alias: str
    fdcan_wheel_alias: str
    sbus_alias: str
    rgb_alias: str
    usb_alias: str


def fail(message: str) -> None:
    raise CodegenError(message)


def run(command: list[str], *, cwd: Path = ROOT) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(command, cwd=cwd, capture_output=True, text=True)
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        fail(f"command failed: {' '.join(command)}\n{detail}")
    return result


def version_tuple(version: str) -> tuple[int, int, int]:
    match = re.match(r"^(\d+)\.(\d+)\.(\d+)", version)
    if match is None:
        fail(f"unsupported version format: {version}")
    return tuple(int(part) for part in match.groups())


def interpreter_for_command(command: str) -> list[str] | None:
    try:
        first_line = Path(command).read_text(encoding="utf-8", errors="ignore").splitlines()[0]
    except (OSError, IndexError):
        return None
    if not first_line.startswith("#!"):
        return None
    interpreter = shlex.split(first_line[2:].strip())
    if not interpreter or interpreter[0].endswith("env"):
        return None
    return interpreter


def installed_package_version(package: str, command: str) -> str:
    try:
        return importlib.metadata.version(package)
    except importlib.metadata.PackageNotFoundError:
        pass

    interpreter = interpreter_for_command(command)
    if interpreter is not None:
        code = (
            "import importlib.metadata; "
            f"print(importlib.metadata.version({package!r}))"
        )
        result = run([*interpreter, "-c", code])
        version = result.stdout.strip()
        if version:
            return version

    result = run([command, "--help"])
    help_text = result.stdout + result.stderr
    match = re.search(rf"\b{re.escape(package)}\s+(\d+\.\d+\.\d+[^\s]*)", help_text)
    if match is not None:
        return match.group(1)
    fail(
        f"cannot determine {package} version; install it in the active Python "
        f"environment with: {sys.executable} -m pip install -r "
        f"{REQUIREMENTS_PATH.relative_to(ROOT)}"
    )


def discover_toolchain() -> Toolchain:
    commands: dict[str, str] = {}
    for name, required_help_tokens in REQUIRED_TOOLS.items():
        executable = shutil.which(name)
        if executable is None:
            fail(
                f"missing required generator '{name}'; install tools with: "
                f"{sys.executable} -m pip install -r {REQUIREMENTS_PATH.relative_to(ROOT)}"
            )
        help_result = run([executable, "--help"])
        help_text = help_result.stdout + help_result.stderr
        missing = [token for token in required_help_tokens if token not in help_text]
        if missing:
            fail(f"{name} lacks required options: {', '.join(missing)}")
        commands[name] = executable

    libxr_version = installed_package_version("libxr", commands["xr_parse_ioc"])
    xrobot_version = installed_package_version("xrobot", commands["xrobot_gen_main"])
    for package, actual in (("libxr", libxr_version), ("xrobot", xrobot_version)):
        minimum = MINIMUM_VERSIONS[package]
        if version_tuple(actual) < version_tuple(minimum):
            fail(f"{package}>={minimum} is required, found {actual}")

    return Toolchain(
        libxr_version=libxr_version,
        xrobot_version=xrobot_version,
        xr_parse_ioc=commands["xr_parse_ioc"],
        xr_gen_code_stm32=commands["xr_gen_code_stm32"],
        xrobot_gen_main=commands["xrobot_gen_main"],
    )


def require_file_tokens(path: Path, tokens: tuple[str, ...]) -> None:
    try:
        display_path = path.relative_to(ROOT)
    except ValueError:
        display_path = path
    if not path.is_file():
        fail(f"required file is missing: {display_path}")
    content = path.read_text(encoding="utf-8", errors="ignore")
    missing = [token for token in tokens if token not in content]
    if missing:
        fail(f"{display_path} lacks required API: {', '.join(missing)}")


def validate_libxr_checkout() -> None:
    for relative_path, tokens in REQUIRED_LIBXR_HEADERS.items():
        require_file_tokens(ROOT / relative_path, tokens)


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        fail(f"cannot load {path.relative_to(ROOT)}: {exc}")
    if not isinstance(data, dict):
        fail(f"{path.relative_to(ROOT)} must contain a YAML mapping")
    return data


def integer(mapping: dict[str, Any], key: str, *, minimum: int = 0) -> int:
    value = mapping.get(key)
    if isinstance(value, bool):
        fail(f"{key} must be an integer")
    try:
        parsed = int(str(value), 0)
    except (TypeError, ValueError):
        fail(f"{key} must be an integer")
    if parsed < minimum:
        fail(f"{key} must be >= {minimum}")
    return parsed


def mapping(parent: dict[str, Any], key: str) -> dict[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        fail(f"missing configuration mapping: {key}")
    return value


def text(mapping_value: dict[str, Any], key: str) -> str:
    value = mapping_value.get(key)
    if not isinstance(value, str) or not value:
        fail(f"{key} must be a non-empty string")
    return value


def device_alias(config: dict[str, Any], device: str, expected_type: str) -> str:
    aliases_root = mapping(config, "device_aliases")
    device_config = mapping(aliases_root, device)
    if device_config.get("type") != expected_type:
        fail(f"device_aliases.{device}.type must be {expected_type}")
    aliases = device_config.get("aliases")
    if not isinstance(aliases, list) or len(aliases) != 1 or not isinstance(aliases[0], str):
        fail(f"device_aliases.{device}.aliases must contain exactly one alias")
    return aliases[0]


def load_platform_config() -> PlatformConfig:
    config = load_yaml(CONFIG_PATH)
    timer = mapping(config, "software_timer")
    fdcan = mapping(config, "FDCAN")
    uart5 = mapping(mapping(config, "USART"), "uart5")
    spi6 = mapping(mapping(config, "SPI"), "spi6")
    usb = mapping(mapping(config, "USB"), "usb_otg_hs")

    queue_sizes = {
        integer(mapping(fdcan, instance), "queue_size", minimum=1)
        for instance in ("FDCAN1", "FDCAN2", "FDCAN3")
    }
    if len(queue_sizes) != 1:
        fail("FDCAN1/2/3 queue_size values must match")

    usb_enabled = usb.get("enable")
    if usb_enabled is not True:
        fail("USB.usb_otg_hs.enable must be true")

    platform = PlatformConfig(
        software_timer_priority=integer(timer, "priority"),
        software_timer_stack=integer(timer, "stack_depth", minimum=1),
        fdcan_queue_size=queue_sizes.pop(),
        sbus_rx_size=integer(uart5, "rx_buffer_size", minimum=1),
        sbus_queue_size=integer(uart5, "tx_queue_size", minimum=1),
        sbus_dma_section=text(uart5, "dma_section"),
        spi_rx_size=integer(spi6, "rx_buffer_size", minimum=1),
        spi_tx_size=integer(spi6, "tx_buffer_size", minimum=1),
        spi_dma_threshold=integer(spi6, "dma_enable_min_size", minimum=1),
        usb_rx_fifo_size=integer(usb, "rx_fifo_size", minimum=1),
        usb_ep0_buffer_size=integer(usb, "ep0_packet_size", minimum=1),
        usb_ep1_out_buffer_size=integer(usb, "rx_buffer_size", minimum=1),
        usb_ep1_in_buffer_size=integer(usb, "tx_buffer_size", minimum=1),
        usb_ep2_in_buffer_size=integer(usb, "notification_buffer_size", minimum=1),
        usb_ep0_in_fifo_size=integer(usb, "ep0_tx_fifo_size", minimum=1),
        usb_ep1_in_fifo_size=integer(usb, "tx_fifo_size", minimum=1),
        usb_ep2_in_fifo_size=integer(usb, "notification_tx_fifo_size", minimum=1),
        usb_cdc_rx_size=integer(usb, "cdc_rx_fifo_size", minimum=1),
        usb_cdc_tx_size=integer(usb, "cdc_tx_fifo_size", minimum=1),
        usb_cdc_queue_size=integer(usb, "cdc_queue_size", minimum=1),
        usb_vid=integer(usb, "vid", minimum=1),
        usb_pid=integer(usb, "pid", minimum=1),
        usb_bcd=integer(usb, "bcd", minimum=1),
        usb_manufacturer=text(usb, "manufacturer"),
        usb_product=text(usb, "product"),
        usb_serial=text(usb, "serial"),
        fdcan_front_alias=device_alias(config, "fdcan1", "FDCAN"),
        fdcan_rear_alias=device_alias(config, "fdcan2", "FDCAN"),
        fdcan_wheel_alias=device_alias(config, "fdcan3", "FDCAN"),
        sbus_alias=device_alias(config, "uart5", "UART"),
        rgb_alias=device_alias(config, "spi6", "SPI"),
        usb_alias=device_alias(config, "usb_otg_hs_cdc", "UART"),
    )

    aliases = (
        platform.fdcan_front_alias,
        platform.fdcan_rear_alias,
        platform.fdcan_wheel_alias,
        platform.sbus_alias,
        platform.rgb_alias,
        platform.usb_alias,
    )
    if len(set(aliases)) != len(aliases):
        fail("hardware aliases must be unique")
    invalid_aliases = [alias for alias in aliases if re.fullmatch(r"[A-Za-z_]\w*", alias) is None]
    if invalid_aliases:
        fail(f"hardware aliases must be valid C++ identifiers: {', '.join(invalid_aliases)}")

    fifo_total = (
        platform.usb_rx_fifo_size
        + platform.usb_ep0_in_fifo_size
        + platform.usb_ep1_in_fifo_size
        + platform.usb_ep2_in_fifo_size
    )
    if fifo_total != 4096:
        fail(f"XRUSB FIFO allocation must total 4096 bytes, found {fifo_total}")
    if platform.sbus_dma_section != ".dma_buffer":
        fail("UART5 RX buffer must use the .dma_buffer section")
    return platform


def parse_ioc(toolchain: Toolchain, temp_root: Path) -> dict[str, Any]:
    input_dir = temp_root / "ioc"
    input_dir.mkdir()
    shutil.copy2(IOC_PATH, input_dir / IOC_PATH.name)
    parsed_path = temp_root / "parsed.yaml"
    run(
        [
            toolchain.xr_parse_ioc,
            "--directory",
            str(input_dir),
            "--output",
            str(parsed_path),
        ]
    )
    return load_yaml(parsed_path)


def peripheral(parsed: dict[str, Any], group: str, instance: str) -> dict[str, Any]:
    peripherals = mapping(parsed, "Peripherals")
    group_config = mapping(peripherals, group)
    return dict(mapping(group_config, instance))


def validate_and_normalize_ioc(parsed: dict[str, Any], platform: PlatformConfig) -> dict[str, Any]:
    ioc_text = IOC_PATH.read_text(encoding="utf-8")
    required_ioc_tokens = (
        "Mcu.IP9=FREERTOS",
        "rtos.0.ip=FREERTOS",
        "PD2.Signal=UART5_RX",
        "Dma.Request0=UART5_RX",
        "Dma.UART5_RX.0.Mode=DMA_CIRCULAR",
        "USB_OTG_HS.VirtualMode-Device_Only_FS=Device_Only_FS",
    )
    missing = [token for token in required_ioc_tokens if token not in ioc_text]
    if missing:
        fail(f"dm02.ioc lacks required configuration: {', '.join(missing)}")
    if "UART5_TX" in ioc_text or "PC12.Signal=UART5" in ioc_text:
        fail("UART5 must be RX-only and PC12 must not be assigned to UART5_TX")

    usb_hal = (ROOT / "Core/Src/usb_otg.c").read_text(encoding="utf-8")
    if "hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;" not in usb_hal:
        fail("USB internal DMA must remain disabled for the configured endpoint buffers")

    uart_hal = (ROOT / "Core/Src/usart.c").read_text(encoding="utf-8")
    if "huart5.Init.Mode = UART_MODE_RX;" not in uart_hal:
        fail("Core/Src/usart.c must be regenerated with UART5 in RX-only mode")

    freertos_config = (ROOT / "Core/Inc/FreeRTOSConfig.h").read_text(encoding="utf-8")
    required_freertos_tokens = (
        "configTOTAL_HEAP_SIZE                    ((size_t)49152)",
        "configUSE_TIMERS                         0",
    )
    missing_freertos = [
        token for token in required_freertos_tokens if token not in freertos_config
    ]
    if missing_freertos:
        fail("FreeRTOSConfig.h no longer matches the production memory configuration")

    uart5 = peripheral(parsed, "USART", "UART5")
    uart_dma = mapping(uart5, "dma")
    uart_rx_dma = mapping(uart_dma, "dma_rx")
    expected_uart = {
        "BaudRate": 100000,
        "WordLength": "WORDLENGTH_9B",
        "Parity": "PARITY_EVEN",
        "StopBits": "UART_STOPBITS_2",
        "DMA_RX": "ENABLE",
    }
    for key, expected in expected_uart.items():
        if uart5.get(key) != expected:
            fail(f"UART5 {key} must be {expected}, found {uart5.get(key)}")
    if uart5.get("DMA_TX") == "ENABLE" or uart_rx_dma.get("mode") != "Circular":
        fail("UART5 must use circular RX DMA without TX DMA")

    spi6 = peripheral(parsed, "SPI", "SPI6")
    if spi6.get("Direction") != "SPI_DIRECTION_2LINES_TXONLY":
        fail("SPI6 must remain TX-only")
    if spi6.get("DMA_RX") == "ENABLE" or spi6.get("DMA_TX") == "ENABLE":
        fail("SPI6 DMA must remain disabled")

    selected_peripherals = {
        "FDCAN": {
            instance: peripheral(parsed, "FDCAN", instance)
            for instance in ("FDCAN1", "FDCAN2", "FDCAN3")
        },
        "SPI": {"SPI6": spi6},
        "USART": {"UART5": uart5},
        "USB": {"USB_OTG_HS": peripheral(parsed, "USB", "USB_OTG_HS")},
    }
    selected_peripherals["USB"]["USB_OTG_HS"]["enable"] = True

    return {
        "GPIO": {},
        "Peripherals": selected_peripherals,
        "DMA": parsed.get("DMA", {}),
        "Timebase": parsed.get("Timebase", {"Source": "SysTick", "IRQ": None}),
        "Mcu": mapping(parsed, "Mcu"),
        "FreeRTOS": {},
        "software_timer": {
            "priority": platform.software_timer_priority,
            "stack_depth": platform.software_timer_stack,
        },
    }


def smoke_test_libxr_emitter(
    toolchain: Toolchain, normalized: dict[str, Any], temp_root: Path
) -> None:
    normalized_path = temp_root / "normalized.yaml"
    normalized_path.write_text(
        yaml.safe_dump(normalized, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    output_dir = temp_root / "official"
    output_dir.mkdir()
    output_path = output_dir / "app_main.cpp"
    run(
        [
            toolchain.xr_gen_code_stm32,
            "--input",
            str(normalized_path),
            "--output",
            str(output_path),
            "--xrobot",
            "--libxr-config",
            str(CONFIG_PATH),
        ]
    )
    require_file_tokens(
        output_path,
        (
            f"PlatformInit({normalized['software_timer']['priority']}, "
            f"{normalized['software_timer']['stack_depth']})",
            "STM32CANFD",
            "STM32UART",
            "STM32SPI",
            "STM32USBDeviceOtgHS",
            "LibXR::USB::CDCUart",
            "LibXR::HardwareContainer",
        ),
    )


def generated_notice(toolchain: Toolchain) -> str:
    return (
        "// AUTO-GENERATED by tools/generate.py. DO NOT EDIT.\n"
        "// Sources: dm02.ioc, User/libxr_config.yaml, User/xrobot.yaml\n"
        f"// Generators: libxr {toolchain.libxr_version}, xrobot {toolchain.xrobot_version}\n"
    )


def cpp_string(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"')


def render_app_main(toolchain: Toolchain, config: PlatformConfig) -> str:
    packet_size_enum = {
        8: "SIZE_8",
        16: "SIZE_16",
        32: "SIZE_32",
        64: "SIZE_64",
    }.get(config.usb_ep0_buffer_size)
    if packet_size_enum is None:
        fail("USB EP0 packet size must be 8, 16, 32, or 64")

    return generated_notice(toolchain) + f'''#include "app_main.h"

#include "cdc_uart.hpp"
#include "fdcan.h"
#include "libxr.hpp"
#include "main.h"
#include "spi.h"
#include "stm32_canfd.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb_dev.hpp"
#include "uart.hpp"
#include "usb_otg.h"
#include "usart.h"
#include "xrobot_main.hpp"

namespace
{{
using LibXR::USB::Endpoint;

constexpr auto kEnglish = LibXR::USB::DescriptorStrings::MakeLanguagePack(
    LibXR::USB::DescriptorStrings::Language::EN_US,
    "{cpp_string(config.usb_manufacturer)}", "{cpp_string(config.usb_product)}",
    "{cpp_string(config.usb_serial)}");

__attribute__((section("{config.sbus_dma_section}"), aligned(32)))
uint8_t sbus_rx[{config.sbus_rx_size}];
alignas(32) uint8_t spi6_rx[{config.spi_rx_size}];
alignas(32) uint8_t spi6_tx[{config.spi_tx_size}];
alignas(32) uint8_t usb_ep0_out[{config.usb_ep0_buffer_size}];
alignas(32) uint8_t usb_ep1_out[{config.usb_ep1_out_buffer_size}];
alignas(32) uint8_t usb_ep0_in[{config.usb_ep0_buffer_size}];
alignas(32) uint8_t usb_ep1_in[{config.usb_ep1_in_buffer_size}];
alignas(32) uint8_t usb_ep2_in[{config.usb_ep2_in_buffer_size}];
}}  // namespace

extern "C" void app_main(void)
{{
  using namespace LibXR;

  STM32Timebase timebase;
  PlatformInit({config.software_timer_priority}, {config.software_timer_stack});

  STM32CANFD {config.fdcan_front_alias}(&hfdcan1, {config.fdcan_queue_size});
  STM32CANFD {config.fdcan_rear_alias}(&hfdcan2, {config.fdcan_queue_size});
  STM32CANFD {config.fdcan_wheel_alias}(&hfdcan3, {config.fdcan_queue_size});
  STM32UART {config.sbus_alias}(&huart5, RawData(sbus_rx), {{nullptr, 0}}, {config.sbus_queue_size});
  STM32SPI {config.rgb_alias}(&hspi6, RawData(spi6_rx), RawData(spi6_tx), {config.spi_dma_threshold});

  USB::CDCUart {config.usb_alias}(
      Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP1,
      Endpoint::EPNumber::EP2, {config.usb_cdc_rx_size},
      {config.usb_cdc_tx_size}, {config.usb_cdc_queue_size});
  STM32USBDeviceOtgHS usb_device(
      &hpcd_USB_OTG_HS, {config.usb_rx_fifo_size},
      {{RawData(usb_ep0_out), RawData(usb_ep1_out)}},
      {{{{RawData(usb_ep0_in), {config.usb_ep0_in_fifo_size}}}, {{RawData(usb_ep1_in), {config.usb_ep1_in_fifo_size}}},
       {{RawData(usb_ep2_in), {config.usb_ep2_in_fifo_size}}}}},
      USB::DeviceDescriptor::PacketSize0::{packet_size_enum},
      0x{config.usb_vid:04X}, 0x{config.usb_pid:04X}, 0x{config.usb_bcd:04X},
      {{&kEnglish}}, {{{{&{config.usb_alias}}}}},
      ConstRawData(reinterpret_cast<const void*>(UID_BASE), 12));

  HardwareContainer hardware(
      Entry<FDCAN>{{{config.fdcan_front_alias}, {{"{config.fdcan_front_alias}"}}}},
      Entry<FDCAN>{{{config.fdcan_rear_alias}, {{"{config.fdcan_rear_alias}"}}}},
      Entry<FDCAN>{{{config.fdcan_wheel_alias}, {{"{config.fdcan_wheel_alias}"}}}},
      Entry<UART>{{{config.sbus_alias}, {{"{config.sbus_alias}"}}}},
      Entry<UART>{{{config.usb_alias}, {{"{config.usb_alias}"}}}},
      Entry<SPI>{{{config.rgb_alias}, {{"{config.rgb_alias}"}}}});

  usb_device.Init(false);
  usb_device.Start(false);
  XRobotMain(hardware);
}}
'''


def render_app_header(toolchain: Toolchain) -> str:
    return generated_notice(toolchain) + '''#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void app_main(void);

#ifdef __cplusplus
}
#endif
'''


def generate_xrobot_main(toolchain: Toolchain, temp_root: Path) -> str:
    output_path = temp_root / "xrobot_main.hpp"
    run(
        [
            toolchain.xrobot_gen_main,
            "--config",
            str(XROBOT_CONFIG_PATH),
            "--output",
            str(output_path),
        ]
    )
    require_file_tokens(
        output_path,
        (
            "static void XRobotMain(LibXR::HardwareContainer &hw)",
            "ApplicationManager appmgr",
            "SbusReceiver_0",
            "DogMotor_0",
            "WheelMotor_0",
            "ObstacleController_0",
            "HostLink_0",
            "RobotControl_0",
            "StatusLED_0",
            "appmgr.MonitorAll()",
        ),
    )
    return generated_notice(toolchain) + output_path.read_text(encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def codegen_input_paths() -> list[Path]:
    paths = [
        IOC_PATH,
        CONFIG_PATH,
        XROBOT_CONFIG_PATH,
        ROOT / "Modules/modules.yaml",
        ROOT / "Modules/sources.yaml",
        ROOT / "Core/Inc/FreeRTOSConfig.h",
        ROOT / "Core/Src/usart.c",
        ROOT / "Core/Src/usb_otg.c",
        Path(__file__).resolve(),
        REQUIREMENTS_PATH,
    ]
    paths.extend(sorted((ROOT / "Modules").glob("*/*.hpp")))
    paths.extend(ROOT / relative_path for relative_path in REQUIRED_LIBXR_HEADERS)
    return paths


def relative(path: Path) -> str:
    return path.resolve().relative_to(ROOT).as_posix()


def libxr_commit() -> str:
    result = run(["git", "-C", str(LIBXR_ROOT), "rev-parse", "HEAD"])
    return result.stdout.strip()


def render_manifest(
    toolchain: Toolchain, generated_content: dict[Path, str]
) -> str:
    input_paths = codegen_input_paths()
    lines = [
        "# AUTO-GENERATED by tools/generate.py. DO NOT EDIT.",
        f'set(RCDOG_CODEGEN_LIBXR_VERSION "{toolchain.libxr_version}")',
        f'set(RCDOG_CODEGEN_XROBOT_VERSION "{toolchain.xrobot_version}")',
        f'set(RCDOG_CODEGEN_LIBXR_COMMIT "{libxr_commit()}")',
        "set(RCDOG_CODEGEN_INPUT_PATHS",
    ]
    lines.extend(f'  "{relative(path)}"' for path in input_paths)
    lines.append(")")
    lines.append("set(RCDOG_CODEGEN_INPUT_SHA256")
    lines.extend(f'  "{sha256(path)}"' for path in input_paths)
    lines.append(")")
    lines.append("set(RCDOG_CODEGEN_OUTPUT_PATHS")
    lines.extend(f'  "{relative(path)}"' for path in GENERATED_PATHS)
    lines.append(")")
    lines.append("set(RCDOG_CODEGEN_OUTPUT_SHA256")
    for path in GENERATED_PATHS:
        digest = hashlib.sha256(generated_content[path].encode("utf-8")).hexdigest()
        lines.append(f'  "{digest}"')
    lines.extend((")", ""))
    return "\n".join(lines)


def atomic_write(path: Path, content: str) -> bool:
    encoded = content.encode("utf-8")
    if path.is_file() and path.read_bytes() == encoded:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)
    return True


def compare_file(path: Path, expected: str) -> str | None:
    if not path.is_file():
        return f"missing generated file: {relative(path)}"
    if path.read_text(encoding="utf-8") != expected:
        return f"stale generated file: {relative(path)}"
    return None


def generate(check_only: bool) -> None:
    toolchain = discover_toolchain()
    validate_libxr_checkout()
    platform = load_platform_config()

    with tempfile.TemporaryDirectory(prefix="rcdog-codegen-") as temp_directory:
        temp_root = Path(temp_directory)
        parsed = parse_ioc(toolchain, temp_root)
        normalized = validate_and_normalize_ioc(parsed, platform)
        smoke_test_libxr_emitter(toolchain, normalized, temp_root)
        generated_content = {
            ROOT / "User/app_main.cpp": render_app_main(toolchain, platform),
            ROOT / "User/app_main.h": render_app_header(toolchain),
            ROOT / "User/xrobot_main.hpp": generate_xrobot_main(toolchain, temp_root),
        }
        manifest = render_manifest(toolchain, generated_content)

    if check_only:
        errors = [
            error
            for path, content in generated_content.items()
            if (error := compare_file(path, content)) is not None
        ]
        manifest_error = compare_file(MANIFEST_PATH, manifest)
        if manifest_error is not None:
            errors.append(manifest_error)
        if errors:
            fail("\n".join(errors) + "\nrun: python3 tools/generate.py")
        print(
            f"generated files are current (libxr {toolchain.libxr_version}, "
            f"xrobot {toolchain.xrobot_version})"
        )
        return

    changed = [
        relative(path)
        for path, content in generated_content.items()
        if atomic_write(path, content)
    ]
    if atomic_write(MANIFEST_PATH, manifest):
        changed.append(relative(MANIFEST_PATH))
    if changed:
        print("updated: " + ", ".join(changed))
    else:
        print("generated files already current")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check", action="store_true", help="verify committed generated files without writing"
    )
    args = parser.parse_args()
    try:
        generate(args.check)
    except CodegenError as exc:
        print(f"code generation failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
