#!/usr/bin/env python3
from aido_schemas import protocol_agent_DB20_fullstate, wrap_direct

from gtduckie import MyFullAgent


def main() -> None:
    node = MyFullAgent()
    protocol = protocol_agent_DB20_fullstate
    wrap_direct(node=node, protocol=protocol)


if __name__ == "__main__":
    main()
