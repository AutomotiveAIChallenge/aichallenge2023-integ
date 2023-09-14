# AI Challenge Documentation

This is the folder that contains the contents for the AI Challenge documentation.

## How to check locally

Install `docfx`.

```
sudo apt install -y dotnet-sdk-7.0
dotnet tool install -g docfx
```

Add to ~/.bashrc

```
export PATH="$PATH:/home/<USERNAME>/.dotnet/tools"
```

Generate the documentation locally. This will create the HTML files under `documentation/docfx_project/_site`, and a server will start on `http://localhost:8080`.
This script must be ran every time there is a change to the documentation markdown files.

```
cd documentation
./make_docs.sh
```

Access `http://localhost:8080` to see documentation.

