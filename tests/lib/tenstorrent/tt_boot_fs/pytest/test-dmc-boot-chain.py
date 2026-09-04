# SPDX-License-Identifier: Apache-2.0

import hashlib
from pathlib import Path
import struct
import sys

from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding, rsa
import pytest

MODULE_ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(MODULE_ROOT / "scripts"))
import verify_dmc_boot_chain as verifier  # noqa: E402


@pytest.fixture(scope="module")
def keys():
    return [
        rsa.generate_private_key(public_exponent=65537, key_size=2048) for _ in range(2)
    ]


def public_der(key):
    return key.public_key().public_bytes(
        serialization.Encoding.DER, serialization.PublicFormat.PKCS1
    )


def bootloader(key):
    return b"bootloader prefix" + public_der(key) + b"bootloader suffix"


def tlv(kind, value):
    return struct.pack("<BBH", kind, 0, len(value)) + value


def signed_image(key, advertised_key=None, protected=False):
    data = b"DMC test payload"
    protection = struct.pack("<HH", 0x6908, 4) if protected else b""
    payload = struct.pack("<IIHHI", 0x96F3B83D, 0, 32, len(protection), len(data))
    payload = payload.ljust(32, b"\0") + data + protection
    signature = key.sign(
        payload,
        padding.PSS(mgf=padding.MGF1(hashes.SHA256()), salt_length=32),
        hashes.SHA256(),
    )
    key_hash = hashlib.sha256(public_der(advertised_key or key)).digest()
    entries = tlv(0x10, hashlib.sha256(payload).digest()) + tlv(0x01, key_hash)
    entries += tlv(0x20, signature)
    return payload + struct.pack("<HH", 0x6907, len(entries) + 4) + entries


@pytest.mark.parametrize("index", [0, 1])
@pytest.mark.parametrize("protected", [False, True])
def test_matching_bootloader_and_image(keys, index, protected):
    result = verifier.verify_dmc_image(
        bootloader(keys[index]), signed_image(keys[index], protected=protected)
    )
    assert result["signature_valid"] is True


def test_production_bootloader_rejects_development_update(keys):
    with pytest.raises(ValueError, match="signing-key mismatch"):
        verifier.verify_dmc_image(bootloader(keys[0]), signed_image(keys[1]))


def test_forged_matching_key_id_does_not_bypass_signature_check(keys):
    with pytest.raises(ValueError, match="signature rejected"):
        verifier.verify_dmc_image(
            bootloader(keys[0]), signed_image(keys[1], advertised_key=keys[0])
        )


def test_payload_corruption(keys):
    image = bytearray(signed_image(keys[0]))
    image[32] ^= 1
    with pytest.raises(ValueError, match="digest mismatch"):
        verifier.verify_dmc_image(bootloader(keys[0]), image)


def test_signature_corruption(keys):
    image = bytearray(signed_image(keys[0]))
    image[-1] ^= 1
    with pytest.raises(ValueError, match="signature rejected"):
        verifier.verify_dmc_image(bootloader(keys[0]), image)


@pytest.mark.parametrize("size", [0, 16, 32, 49, 64, 100, 300])
def test_truncated_image(keys, size):
    with pytest.raises(ValueError):
        verifier.verify_dmc_image(bootloader(keys[0]), signed_image(keys[0])[:size])


def test_missing_bootloader_key(keys):
    with pytest.raises(ValueError, match="exactly one"):
        verifier.verify_dmc_image(b"not a bootloader", signed_image(keys[0]))


def test_ambiguous_bootloader_keys(keys):
    with pytest.raises(ValueError, match="exactly one"):
        verifier.verify_dmc_image(
            bootloader(keys[0]) + bootloader(keys[1]), signed_image(keys[0])
        )
