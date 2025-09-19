# Contributing to PyRPOD

Please read carefully before contributing to PyRPOD.

## Code Contributions

To contribute to the PyRPOD source code, you must agree to the [Developer Certificate of Origin (DCO)](https://developercertificate.org/).

By submitting a pull request, you implicitly agree to the certifications and terms of the DCO. No additional actions are required.

We aim to make our contribution process as developer-friendly as possible while maintaining high-quality standards.

## Bug Reports

If you identify a bug, please open an issue in the repository. Provide detailed information, including:

- Steps to reproduce the issue.
- Expected versus actual behavior.
- Any relevant code snippets or configurations.

Reports should ideally include minimal code examples and associated outputs to make troubleshooting easier.

## Feature Requests

Feature requests can be submitted as issues. While we strive to accommodate requests, keep in mind that PyRPOD is an open-source project with limited resources. We may provide guidance on implementing your requested features.

## Contributions of Novel Features

PyRPOD accepts contributions from community members with demonstrated programming skills and familiarity with the PyRPOD framework. Before starting significant development efforts:

1. Review the PyRPOD documentation and style guides.
2. Engage with maintainers and the community to validate your proposal's alignment with PyRPOD's goals.

To contribute:

1. Fork the repository following the [forking workflow guide](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/working-with-forks).
2. Create new modules, functions, or classes instead of altering existing ones wherever possible.
3. Document your additions thoroughly, including:
   - Code comments.
   - References to academic papers if applicable.
   - Example usage or test cases.

When your contribution is ready, submit a pull request. Include a clear summary of your changes, their purpose, and any critical details.

## Testing and Continuous Integration (CI)

PyRPOD uses automated testing to ensure stability and prevent regressions. To facilitate this:

1. Write unit tests for your contributions in the `tests` directory.
2. Ensure all tests pass before submitting a pull request.
3. If possible, run CI tests locally before pushing your changes. Follow the instructions in the project’s CI setup documentation.

## Code Formatting

Consistency is key. PyRPOD enforces uniform code formatting using the `black` code formatter.

- Install and run `black` on your code before committing:
  ```bash
  black .
  ```
- The CI pipeline will reject pull requests with improperly formatted code.

## Licensing

By contributing, you agree that your contributions will be licensed under the same license as PyRPOD (currently [GPL-3.0 License](LICENSE.md)). Ensure your contributions comply with third-party licensing terms if applicable.

---

Thank you for contributing to PyRPOD! Your efforts help make this project a valuable resource for the community.
