# Viam Hackathon

This repository contains the saved code modules and workflow configuration for a Viam robotic mail-handling and sorting demo using a robotic arm, camera, and gripper.

## Workflow

The `main` service coordinates the sequence:

1. Move to the configured mailbox handle pose and run `mail-handler` to open or handle the mailbox.
2. Move to the mail reading pose; `mail-address-reader` captures and reads the mail.
3. `sorter` classifies using the configured `receiver` section and `address` field (the saved condition is `Boston`).
4. Pick the mail, lift it, move to the bins view pose, offset laterally for the selected group, and release it.

The workflow uses color detection and depth-based segmentation for letters and green bins. See [config/viam-workflow-config.json](config/viam-workflow-config.json) for the saved service settings and joint poses. Hardware identity, network addresses, and camera serial numbers are omitted from that snapshot.

## Repository layout

- `main.py` — existing repository entry point.
- `services/main.py` — Viam workflow orchestrator.
- `services/mail-handler.py` — mailbox handle interaction.
- `services/mail-address-reader.py` — mail address reading.
- `services/sorter.py` — recipient-based sorting logic.
- `services/code-1.py`, `services/code-2.py` — saved helper services.
- `config/viam-workflow-config.json` — sanitized Viam settings and saved poses.

The custom service sources are copied from the Viam app. Their dependencies and hardware configuration are specific to the original Viam setup.
