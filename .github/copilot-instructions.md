# Global Copilot Instructions

## Response Style
- Provide code/logic only - no documentation, summaries, or explanations unless explicitly asked
- Skip introductory/concluding remarks
- Answer with minimal text overhead
- Prioritize speed and conciseness

## IDE Integration
- Use PyCharm's built-in features (refactor, run, debug, test runners)
- Never suggest terminal commands for tasks PyCharm can handle natively
- Leverage IDE's build/compile/test/run configurations
- Use IDE shortcuts and built-in tools over CLI

## Code Generation
- Generate implementation code directly
- Skip docstrings, comments, and type hints unless requested
- Focus on logic and functionality
- Omit README files, setup scripts, or build instructions

## Prohibited
- No documentation generation
- No file creation via terminal (use IDE file creation)
- No build commands (use IDE build system)
- No verbose explanations
- No "here's what this does" preambles