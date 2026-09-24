# AGENTS.md

## README Changes

- Do not modify `README.md` without the user's explicit instruction.

## ROS Integration Testing

- Use the Visual Studio Docker test environments defined in `src/testEnvironments.json`.
- Reuse existing `rclnet-tests:<distro>` images and Docker build cache. Rebuild only when the corresponding Dockerfile or test interface package changes.
- Test images include the .NET 10 SDK and the .NET 8/9 runtimes. Run tests inside the reusable container so the ROS and .NET environments match.
- Do not remove reusable `rclnet-tests:*` images as routine cleanup.
- If the Visual Studio test environment cannot be used, run `dotnet test` inside the corresponding reusable test image instead of introducing another test runner.

## Test Code Comments

- Do not add XML documentation comments to test code. Use ordinary comments only when they explain important or complex logic.

## C# Code Style

- Use braces for `if` and `else` bodies, including single-statement bodies, except for the repetitive short inline cases described below. Keep `else if` chains readable.
- Put opening and closing braces on separate lines; do not use single-line blocks. Apply this to control flow, methods, constructors, custom accessor bodies, and block-bodied lambdas. Auto-properties without custom getter/setter/init bodies may remain inline, including accessor visibility modifiers and property initializers (for example, `public int Count { get; private set; }`).
- A large, repetitive sequence of short `if ... else ...` cases may remain inline only when the logic is clear and readability is preserved; braces may be omitted for these single-statement branches (for example, `if (adding) _cTimers++; else _cTimers--;`). This is a narrow manual exception to the default formatter/analyzer rules; ordinary branches still require braces.
- `try`, `catch`, `finally`, `lock`, `while`, `do`, `for`, `foreach`, `using`, and `fixed` statements must use braces with multiline bodies, even for an empty body or a single statement. Do not apply the repetitive short `if/else` exception to these constructs.
- Separate consecutive code blocks with a blank line. Keep consecutive simple statements together as one logical block, splitting them with blank lines when their responsibilities differ. Do not insert a blank line between every simple statement.
- Prefer the appropriate exception `ThrowIf` helper over a conditional `throw new` when supported by all target frameworks and when the validation semantics remain equivalent. Preserve parameter names and identify the disposed object type correctly; retain explicit throws when no suitable helper exists or a specific diagnostic message is needed.
- Apply these conventions to production and test code. EditorConfig handles mechanical formatting; logical statement grouping still requires manual review.
