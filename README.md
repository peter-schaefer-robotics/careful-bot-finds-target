# Control Barrier Functions on a 2-link arm

An interactive, dependency-free explanation of control barrier functions. A planar 2-link arm
drives toward a target you click while a CBF safety filter keeps its end effector out of the
forbidden regions you draw. Simple mode is the default; Expert mode reveals the derivations, one more parameter and the
approach ruler on the canvas.

The whole lesson is five formulas, and the five numbered sections of
[`public/js/cbf.js`](public/js/cbf.js) are those five steps in the same order.

## Develop

```bash
npm install
npm run dev      # http://localhost:4321
npm run build    # writes docs/
```

## Deploy

GitHub Pages, **Source: branch `main`, folder `/docs`**. Astro builds straight into `docs/`, and that
folder is committed — the published site *is* the committed build, so **run `npm run build` before
committing**. `public/.nojekyll` keeps Pages from running the output through Jekyll.

### Why there is no `base` path

`astro.config.mjs` deliberately sets neither `site` nor `base`. Every asset is referenced with a
relative URL, so the same `docs/` works at `user.github.io/` and under any repository name. Three
invariants keep that true:

1. **One page, at the root.** Relative URLs only resolve correctly from `/index.html`.
2. **Everything the browser loads lives in `public/`** and is referenced with `./…`, never a leading `/`.
3. **The one `<script>` tag carries `is:inline`.** Without it Astro rewrites it to a hashed bundle
   path, which 404s under a repository sub-path *and* destroys the view-source value.

Verify all three in one go:

```bash
npm run build
grep -q '/_astro/' docs/index.html && echo BROKEN || echo OK
npx http-server . -p 8080     # then open http://localhost:8080/docs/
```

## Layout

```
public/js/cbf.js     the maths — no DOM. §1 kinematics · §2 Jacobian · §3 barrier ·
                     §4 nominal controller · §5 the CBF rows and the filter QP
public/js/app.js     the page — state, pointer input, the fixed-step loop, the canvas
public/css/app.css   tokens, layout, and the one rule that is all of Expert mode
src/                 the Astro shell and the five lesson blocks
```

The JavaScript is shipped unminified and unbundled on purpose: reading it is part of the lesson.
