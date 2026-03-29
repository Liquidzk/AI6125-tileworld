# AI6125-tileworld

## Run

put mason.jar into libs
- `libs/MASON_14.jar`


```
mkdir out/
javac -cp "libs\MASON_14.jar" -d out (Get-ChildItem -Recurse src -Filter *.java | % FullName)
```

Run the random 10-seeds benchmark:

```powershell
java -cp "out;libs\MASON_14.jar" tileworld.TileworldMain
```

Run the GUI:

```powershell
java -cp "out;libs\MASON_14.jar" tileworld.TWGUI
```

Run with `80x80`:

```powershell
java "-Dtileworld.profile=config2" -cp "out;libs\MASON_14.jar" tileworld.TileworldMain
```

## Sample Score

Current `6-agent` version, `5000` steps, fixed seed `4162012`, single run:

| Environment | Map Size | Score |
|---|---:|---:|
| Environment 1 | `50 x 50` | `590` |
| Environment 2 | `80 x 80` | `980` |
