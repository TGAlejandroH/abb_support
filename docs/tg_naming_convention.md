# The `TG` Naming Convention

`TG` (TetraGen) is an **ownership namespace prefix** — a poor-man's namespace for
languages that don't have one. Every identifier we author carries it, so a glance
at a call stack, a pendant program list or a signal table — or a single grep —
tells you what is ours and what was already on the controller.

It marks authorship only. It is not a type tag, not an abbreviation of the thing
it names, and not decoration.

## Rules

| Kind | Form | Examples |
|---|---|---|
| Modules, global-scope entities, I/O signals | `TG_` + PascalCase | `TG_Comms`, `TG_CamOpen`, `doTG_Camera` |
| Routines (PROC / FUNC / TRAP) | `tg` + PascalCase, no separator | `tgMainCycle`, `tgTryUnload`, `tgCycleAbort` |
| Data / variables | `<typePrefix>` + `TG_` + PascalCase | `stTG_ProgName`, `nTG_TravelSpeed`, `tTG_Weld` |

Load-bearing detail on the third row: the platform's own type prefix stays
**first**, the owner tag slots in **after** it. Type-then-owner, so RAPID's
Hungarian prefixes and the pendant's alphabetical grouping by type both survive.

Type prefixes in use: `n` num, `st` string, `b` bool, `t` tooldata,
`wobj` wobjdata, `sd` seamdata, `wd` weavedata, `do`/`di` signals.

Module-local helpers may insert a short module tag after `tg`
(`tgfsSettle` in `TGToolFrameSet_Mod`) to keep local scaffolding out of the
global name space visually as well as lexically.

## Rules of engagement

- Apply it to **everything we create**. Never to anything pre-existing.
- Never rename an existing symbol to add it; never strip it from one of ours.
- It does not excuse a vague name — `tgTryUnload` still has to say what it does.
- Exempt: machine-generated identifiers we don't author, e.g. robtargets carried
  over from a `.tgs` export (`rtW2Start`).

## Why it earns its keep on retrofits

- **Provenance at a glance.** Ownership is visible without opening anything.
- **Collision-proof.** You can't clash with an OEM symbol you've never seen, in a
  language with no namespaces or import scoping.
- **Safe removal.** Decommissioning is mechanical: `TG*` goes, everything else
  stays. This is the property that matters most when handing the cell back.
- **Auditable diff.** The customer can confirm we touched only our own surface.
