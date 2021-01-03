#!/usr/bin/env python3
# simple boardd wrapper that updates the panda first
import os
import time
from selfdrive.kegman_conf import kegman_conf
from setproctitle import setproctitle

from panda import BASEDIR as PANDA_BASEDIR, Panda, PandaDFU, build_st
from common.basedir import BASEDIR
from common.gpio import gpio_init, gpio_set
from selfdrive.hardware import TICI
from selfdrive.hardware.tici.pins import GPIO_HUB_RST_N, GPIO_STM_BOOT0, GPIO_STM_RST_N
from selfdrive.swaglog import cloudlog

def set_panda_power(power=True):
  if not TICI:
    return

  gpio_init(GPIO_STM_RST_N, True)
  gpio_init(GPIO_STM_BOOT0, True)

  gpio_set(GPIO_STM_RST_N, False if is_legacy_panda_reset() else True)
  gpio_set(GPIO_HUB_RST_N, True)

  time.sleep(0.1)

  gpio_set(GPIO_STM_RST_N, power if is_legacy_panda_reset() else (not power))


def get_firmware_fn():
  signed_fn = os.path.join(PANDA_BASEDIR, "board", "obj", "panda.bin.signed")
  if os.path.exists(signed_fn):
    cloudlog.info("Using prebuilt signed firmware")
    return signed_fn
  else:
    cloudlog.info("Building panda firmware")
    fn = "obj/panda.bin"
    build_st(fn, clean=False)
    return os.path.join(PANDA_BASEDIR, "board", fn)


def get_expected_signature(fw_fn=None):
  if fw_fn is None:
    fw_fn = get_firmware_fn()

  try:
    return Panda.get_signature_from_firmware(fw_fn)
  except Exception:
    cloudlog.exception("Error computing expected signature")
    return b""

def update_panda_op():
  panda = None
  panda_dfu = None

  cloudlog.info("Connecting to panda")

  while True:
    # break on normal mode Panda
    panda_list = Panda.list()
    if len(panda_list) > 0:
      cloudlog.info("Panda found, connecting")
      panda = Panda(panda_list[0])
      break

    # flash on DFU mode Panda
    panda_dfu = PandaDFU.list()
    if len(panda_dfu) > 0:
      cloudlog.info("Panda in DFU mode found, flashing recovery")
      panda_dfu = PandaDFU(panda_dfu[0])
      panda_dfu.recover()

    time.sleep(1)

  fw_fn = get_firmware_fn()
  fw_signature = get_expected_signature(fw_fn)

  try:
    serial = panda.get_serial()[0].decode("utf-8")
  except Exception:
    serial = None

  panda_version = "bootstub" if panda.bootstub else panda.get_version()
  panda_signature = b"" if panda.bootstub else panda.get_signature()
  cloudlog.warning("Panda %s connected, version: %s, signature %s, expected %s" % (
    serial,
    panda_version,
    panda_signature.hex(),
    fw_signature.hex(),
  ))

  if panda.bootstub or panda_signature != fw_signature:
    cloudlog.info("Panda firmware out of date, update required")
    panda.flash(fw_fn)
    cloudlog.info("Done flashing")

  if panda.bootstub:
    bootstub_version = panda.get_version()
    cloudlog.info(f"Flashed firmware not booting, flashing development bootloader. Bootstub version: {bootstub_version}")
    panda.recover()
    cloudlog.info("Done flashing bootloader")

  if panda.bootstub:
    cloudlog.info("Panda still not booting, exiting")
    raise AssertionError

  panda_signature = panda.get_signature()
  if panda_signature != fw_signature:
    cloudlog.info("Version mismatch after flashing, exiting")
    raise AssertionError


def update_panda():
  with open(os.path.join(BASEDIR, "VERSION")) as f:
    repo_version = f.read()
  repo_version += "-EON" if os.path.isfile('/EON') else "-DEV"

  panda = None
  panda_dfu = None

  print("Connecting to panda")

  while True:
    # break on normal mode Panda
    panda_list = Panda.list()
    for i in range(len(panda_list)):
    #if len(panda_list) > 0:
      print("Panda found, connecting")
      panda = Panda(panda_list[i])
      if panda.is_white():
        print("white", panda.health())
      elif panda.is_grey():
        print("gray", panda.health())
      else:
        print("black", panda.health())

    if len(panda_list) > 0: break
    
    # flash on DFU mode Panda
    panda_dfu = PandaDFU.list()
    if len(panda_dfu) > 0:
      print("Panda in DFU mode found, flashing recovery")
      panda_dfu = PandaDFU(panda_dfu[0])
      panda_dfu.recover()

    #print("waiting for board..."
    time.sleep(1)

  current_version = "bootstub" if panda.bootstub else str(panda.get_version())
  print("Panda connected, version: %s, expected %s" % (current_version, repo_version))

  if panda.is_white():
    print("white", panda.health())
  elif panda.is_grey():
    print("gray", panda.health())
  else:
    print("black", panda.health())


  if panda.bootstub or not current_version.startswith(repo_version):
    print("Panda firmware out of date, update required")

    if panda.is_black() or (panda.health()['current'] > 2000):
      signed_fn = os.path.join(BASEDIR, "board", "obj", "panda.bin.signed")
      if os.path.exists(signed_fn):
        print("Flashing signed firmware")
        panda.flash(fn=signed_fn)
      else:
        print("Building and flashing unsigned firmware")
        panda.flash()

      print("Done flashing")

  if panda.bootstub:
    if panda.is_black() or (panda.health()['current'] > 2000):
      print("Flashed firmware not booting, flashing development bootloader")
      panda.recover()
      print("Done flashing bootloader")

  if panda.bootstub:
    if panda.is_black() or (panda.health()['current'] > 2000):
      print("Panda still not booting, exiting")
      raise AssertionError

  version = str(panda.get_version())
  if not version.startswith(repo_version):
    if panda.is_black() or (panda.health()['current'] > 2000):
      print("Version mismatch after flashing, exiting")
      raise AssertionError


def main(gctx=None):
  setproctitle('pandad')
  try:
    kegman = kegman_conf()  #.read_config()
    if bool(int(kegman.conf['useAutoFlash'])): 
      update_panda()
  except:
    pass
  #update_panda()
  os.chdir("selfdrive/boardd")
  os.execvp("./boardd", ["./boardd"])

if __name__ == "__main__":
  main()
