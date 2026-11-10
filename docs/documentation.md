# Working on the documentation

The website is built with MkDocs and Material for MkDocs. Its dependencies are
kept in the `docs` dependency group in `pyproject.toml`; installing this group
does not compile Tissu.

## Preview locally

From the repository root, create and activate a virtual environment, then run:

```bash
uv pip install --group docs
mkdocs serve
```

Open the address printed by MkDocs. Changes to Markdown and configuration files
reload the preview automatically.

## Validate

```bash
mkdocs build --strict
```

The build writes the website to the ignored `site/` directory. Warnings about
missing navigation entries, local links, anchors and assets fail the build.
External URLs are not checked by MkDocs.

When adding a page, register it in `mkdocs.yml`. Placeholder pages can be listed
under `exclude_docs` until they are ready to publish.

Math is rendered with `pymdownx.arithmatex` and MathJax. The configuration lives
in `mkdocs.yml` and `docs/javascripts/mathjax.js`.

## Publishing

The **Documentation** workflow validates changes in pull requests and uploads a
preview artifact. Pushes to `main` build and deploy the site to GitHub Pages.
Only the deployment job has `pages: write` and `id-token: write` permissions.

The published address is
[evanrock520-ciencias.github.io/Tissu](https://evanrock520-ciencias.github.io/Tissu/).
For a fork, set the repository's Pages publishing source to **GitHub Actions**
and update `site_url` and `repo_url` in `mkdocs.yml`.
