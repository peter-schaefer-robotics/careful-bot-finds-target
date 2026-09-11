import { defineConfig } from 'astro/config';

// No `site`, no `base` — on purpose.
// Every asset is referenced with a RELATIVE url (./css/app.css, ./js/app.js), so the built
// output works at https://user.github.io/ and at https://user.github.io/any-repo-name/ alike.
// Setting `base` would tie the build to one repo name.
export default defineConfig({
  output: 'static',
  outDir: './docs', // GitHub Pages -> Source: branch main, folder /docs
});
