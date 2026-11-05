#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Print the known-answer vectors used by src/main.c, computed by py-opendisplay.

pip install py-opendisplay==7.18.1
"""

from opendisplay import crypto

master = bytes(range(16))
server_nonce = bytes(range(0x10, 0x20))
client_nonce = bytes(range(0x20, 0x30))
key = crypto.derive_session_key(master, client_nonce, server_nonce)
sid = crypto.derive_session_id(key, client_nonce, server_nonce)

vectors = {
    "kat_mac": crypto.compute_challenge_response(master, server_nonce, client_nonce),
    "kat_session_key": key,
    "kat_session_id": sid,
    "kat_proof": crypto.compute_server_proof(key, server_nonce, client_nonce),
    "kat_frame": crypto.encrypt_command(key, sid, 5, b"\x00\x41", b"\xaa\xbb\xcc"),
    "kat_psk": crypto.aes_cmac(master, b"opendisplay-tls-psk"),
}

print("/* py-opendisplay crypto, master 00..0f, server nonce 10..1f, client nonce 20..2f */")
for name, v in vectors.items():
    print(f"static const uint8_t {name}[] = {{")
    for i in range(0, len(v), 12):
        print("\t" + " ".join(f"0x{b:02x}," for b in v[i : i + 12]))
    print("};")
