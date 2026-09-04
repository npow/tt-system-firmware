#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Offline DMC update authorization check; never opens or resets a device.

The bootloader bundle must match the installed DMC bootloader, independently
established by deployment provenance/readback. A candidate bundle's bootloader
is not evidence that that bootloader is installed. This checks RSA-2048 MCUboot
authorization, not board startup timing, recovery behavior, or load stability.
"""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import tempfile

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding, rsa


def bootloader_public_key(bootloader):
    """Locate the single canonical PKCS1 RSA-2048 key used by this boot format."""
    prefix = bytes.fromhex("3082010a")
    keys = []
    offset = 0
    while (offset := bootloader.find(prefix, offset)) >= 0:
        der = bootloader[offset : offset + 270]
        offset += 1
        try:
            key = serialization.load_der_public_key(der)
        except ValueError:
            continue
        if not isinstance(key, rsa.RSAPublicKey) or key.key_size != 2048:
            continue
        canonical = key.public_bytes(
            serialization.Encoding.DER, serialization.PublicFormat.PKCS1
        )
        if canonical == der:
            keys.append((key, der))
    if len(keys) != 1:
        raise ValueError("expected exactly one embedded RSA-2048 bootloader key")
    return keys[0]


def image_tlvs(image):
    if len(image) < 32:
        raise ValueError("truncated MCUboot image header")
    magic, _, header_size, protected_size, image_size = struct.unpack_from(
        "<IIHHI", image
    )
    if magic != 0x96F3B83D or header_size < 32:
        raise ValueError("invalid MCUboot image header")
    offset = header_size + image_size
    if protected_size:
        if offset + 4 > len(image):
            raise ValueError("missing protected TLVs")
        if struct.unpack_from("<HH", image, offset) != (0x6908, protected_size):
            raise ValueError("invalid protected TLV header")
        offset += protected_size
    if offset + 4 > len(image):
        raise ValueError("missing image TLVs")
    magic, total = struct.unpack_from("<HH", image, offset)
    end = offset + total
    if magic != 0x6907 or total < 4 or end > len(image):
        raise ValueError("invalid image TLV bounds")
    payload = image[:offset]
    offset += 4
    tlvs = {}
    while offset < end:
        if offset + 4 > end:
            raise ValueError("truncated TLV header")
        kind, reserved, size = struct.unpack_from("<BBH", image, offset)
        offset += 4
        if reserved or offset + size > end or kind in tlvs:
            raise ValueError("invalid, truncated or duplicate TLV")
        tlvs[kind] = image[offset : offset + size]
        offset += size
    return payload, tlvs


def verify_dmc_image(bootloader, image):
    key, der = bootloader_public_key(bootloader)
    payload, tlvs = image_tlvs(image)
    key_hash = hashlib.sha256(der).digest()
    if tlvs.get(0x01) != key_hash:
        raise ValueError("DMC signing-key mismatch with supplied bootloader")
    if tlvs.get(0x10) != hashlib.sha256(payload).digest():
        raise ValueError("DMC image digest mismatch")
    signature = tlvs.get(0x20, b"")
    if len(signature) != 256:
        raise ValueError("missing RSA-2048 signature")
    try:
        key.verify(
            signature,
            payload,
            padding.PSS(mgf=padding.MGF1(hashes.SHA256()), salt_length=32),
            hashes.SHA256(),
        )
    except InvalidSignature as exc:
        raise ValueError("DMC signature rejected by supplied bootloader key") from exc
    return {
        "signing_key_sha256": key_hash.hex(),
        "bootloader_sha256": hashlib.sha256(bootloader).hexdigest(),
        "dmc_image_sha256": hashlib.sha256(image).hexdigest(),
        "signature_valid": True,
    }


def verify_bundles(bootloader_bundle, candidate_bundle, board):
    # Reuse the repository's extraction logic rather than a second bootfs parser.
    from tt_fwbundle import extract_bundle_binary

    with tempfile.TemporaryDirectory(prefix="tt-dmc-verify-") as directory:
        bootloader = Path(directory) / "bootloader.bin"
        image = Path(directory) / "dmc.bin"
        extract_bundle_binary(bootloader_bundle, board, "blupdate", bootloader)
        extract_bundle_binary(candidate_bundle, board, "dmfwimg", image)
        return verify_dmc_image(bootloader.read_bytes(), image.read_bytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bootloader-bundle", type=Path, required=True)
    parser.add_argument("--candidate-bundle", type=Path, required=True)
    parser.add_argument(
        "--board", required=True, help="bundle board name, e.g. P150A-1"
    )
    args = parser.parse_args()
    try:
        report = verify_bundles(
            args.bootloader_bundle, args.candidate_bundle, args.board
        )
    except (ValueError, OSError) as exc:
        parser.exit(1, f"DMC boot-chain verification failed: {exc}\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
