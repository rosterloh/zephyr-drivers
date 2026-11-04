# Regenerates the known-answer vectors in src/noise.c (needs aioesphomeapi installed).
from noise.connection import NoiseConnection, Keypair
from aioesphomeapi._frame_helper.noise_encryption import ESPHOME_NOISE_BACKEND
psk = bytes(range(32))
ie = bytes([0x11]*32); re = bytes([0x22]*32)
def mk(init, e):
    p = NoiseConnection.from_name(b"Noise_NNpsk0_25519_ChaChaPoly_SHA256", backend=ESPHOME_NOISE_BACKEND)
    p.set_as_initiator() if init else p.set_as_responder()
    p.set_psks(psk); p.set_prologue(b"NoiseAPIInit\x00\x00")
    p.set_keypair_from_private_bytes(Keypair.EPHEMERAL, e)
    p.start_handshake(); return p
i = mk(True, ie); r = mk(False, re)
m1 = i.write_message(); r.read_message(m1)
m2 = r.write_message(); i.read_message(m2)
c2s = i.encrypt(b"\x00\x07\x00\x00")     # PingRequest, empty
s2c = r.encrypt(b"\x00\x08\x00\x00")     # PingResponse
def c(name, b): print(f"static const uint8_t {name}[] = {{" + ", ".join(f"0x{x:02x}" for x in b) + "};")
for n,b in [("psk",psk),("ie_priv",ie),("re_priv",re),("msg1",m1),("msg2",m2),("c2s",c2s),("s2c",s2c)]: c(n,b)
