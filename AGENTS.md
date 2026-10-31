# Project Engineering Rules (ub-mode by Robert C. Martin / Uncle Bob)

All AI agents working on this project (Antigravity, OpenCode, Claude Code, Cursor) must automatically follow the **ub-mode** disciplined software engineering principles on every code modification.

---

## 1. First Make it Work

### Laziness Protocol & Minimal Edits
* One decision lives in one place.
* Ship the smallest edit that solves the problem. Do not make unrelated refactorings or introduce bloated abstractions.

### Foundational Thinking & Data Flow
* Get the data shape right before writing or modifying logic.
* If a concurrent writer or shared state would cause race conditions, eliminate shared mutable state.

### TDD & Verification
* Write the failing test or reproduction script **before** changing the code.
* Verify that it fails for the exact reason expected.
* Make the smallest change that makes it pass.
* If an honest, meaningful test is impractical, do not write a fake or hollow test.

### Boundary Discipline & Domain Modeling
* Check and validate data strictly at the system boundaries (where it enters from serial, network, user input).
* Inside the system, trust validated data and keep business logic in pure, predictable functions.
* Make illegal states unrepresentable in data structures.
* Make operations idempotent (re-running or recovering from a crash halfway through ends in the same safe state).

---

## 2. Then Make it Right

### Fix Root Causes (No Masking)
* Reproduce the issue first.
* Fix the true root cause.
* **A guard, `try/except: pass`, or `if (val != null)` that only hides the crash without addressing why bad data occurred is strictly forbidden and is NOT a fix.**

### Test Behavior, Not Implementation
* Tests must verify external behavior and real contracts.
* **If a test would still pass when every function returned nothing (void/None), that test is useless and must be removed or rewritten.**

### Clean Code & Quality Gates
* Keep functions small, focused, and below cyclomatic complexity threshold (CRAP < 10).
* Eliminate duplicate code and logic patterns (structural similarity < 0.8).
* Cover edge cases and eliminate dead or unreachable code.
* *(Note: If CLI tools `crapper`, `dryer`, or `mutator` are not installed in the environment, enforce these quality principles directly through careful code design).*

---

## 3. Follow Practice

### Clarity & Atomic Commits
* Explain the reasoning: state the root cause, what changed, and how it was verified.
* Keep git commits small, ordered, and atomic.
* Preserve project stability and documentation integrity.
