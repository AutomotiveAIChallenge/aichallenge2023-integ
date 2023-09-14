docfx docfx_project/docfx.json
docfx docfx_project_en/docfx.json
mkdir docfx_project/_site/en
cp -r docfx_project_en/_site/* docfx_project/_site/en/

docfx docfx_project/docfx.json --serve