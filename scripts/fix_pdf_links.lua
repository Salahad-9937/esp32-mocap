-- Pandoc Lua filter to:
-- 1. Rewrite cross-file Markdown links into internal PDF anchors.
-- 2. Convert HTML comments like <!-- pagebreak --> or <!-- \newpage --> into LaTeX \newpage.

function RawBlock(el)
  if el.format == "html" and (el.text:find("pagebreak") or el.text:find("newpage")) then
    return pandoc.RawBlock("latex", "\\newpage")
  end
end

function RawInline(el)
  if el.format == "html" and (el.text:find("pagebreak") or el.text:find("newpage")) then
    return pandoc.RawInline("latex", "\\newpage")
  end
end

function Link(el)
  if el.target:find("%.md") then
    if el.target:find("#") then
      -- If link contains explicit hash anchor, strip file path
      el.target = el.target:gsub("^.*#", "#")
    else
      -- Extract filename base and convert to anchor format (#sec-filename)
      local filename = el.target:match("([^/]+)%.md$")
      if filename then
        el.target = "#sec-" .. filename
      end
    end
  end
  return el
end