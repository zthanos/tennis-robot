"""Control-panel console package.

Layered architecture for the local web console (see docs and CLAUDE.md):

    HTTP (server.ControlPanelHandler)  -- parse/validate/route/format only
        -> ConsoleApp (app)            -- use-case orchestration
            -> Services (services)     -- one capability each, own their I/O
                Ros / Survey / Path / Camera / Database

Everything is wired by dependency injection in scripts/control_panel.py; there
is no global or class-level mutable state.
"""
