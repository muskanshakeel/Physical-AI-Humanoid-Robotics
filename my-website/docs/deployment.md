---
sidebar_position: 5
---

# Deploy your site

This document describes how to deploy the Docusaurus site to GitHub Pages.

## Prerequisites

- GitHub repository created.
- `my-website` Docusaurus site is initialized.

## Deployment

The `docusaurus.config.ts` file is already configured for GitHub Pages deployment. You will need to update the `url`, `baseUrl`, `organizationName`, and `projectName` fields with your own information.

Once the configuration is updated, you can deploy the site by running the following command:

```bash
npm run deploy
```

This command will build the site and push the `gh-pages` branch to GitHub. You may need to configure your GitHub repository to serve the site from the `gh-pages` branch.
