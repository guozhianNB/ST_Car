#!/usr/bin/env python3
"""Small non-interactive SSH/SFTP helper for the bench MaixCAM."""

from __future__ import annotations

import argparse
from pathlib import Path

import paramiko


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="10.214.237.1")
    parser.add_argument("--user", default="root")
    parser.add_argument("--password", default="root")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--exec", dest="command")
    group.add_argument("--get", nargs=2, metavar=("REMOTE", "LOCAL"))
    group.add_argument("--put", nargs=2, metavar=("LOCAL", "REMOTE"))
    args = parser.parse_args()

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(args.host, username=args.user, password=args.password,
                   timeout=5.0, banner_timeout=5.0, auth_timeout=5.0)
    try:
        if args.command is not None:
            _, stdout, stderr = client.exec_command(args.command, timeout=20.0)
            out = stdout.read().decode("utf-8", errors="replace")
            err = stderr.read().decode("utf-8", errors="replace")
            if out:
                print(out, end="")
            if err:
                print(err, end="")
            return stdout.channel.recv_exit_status()

        sftp = client.open_sftp()
        try:
            if args.get is not None:
                remote, local = args.get
                Path(local).parent.mkdir(parents=True, exist_ok=True)
                sftp.get(remote, local)
            else:
                assert args.put is not None
                local, remote = args.put
                sftp.put(local, remote)
        finally:
            sftp.close()
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
