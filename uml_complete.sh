#!/bin/bash
echo "🚀 Génération complète UML pour RobotController..."

# 1. Générer DOT
~/.venv/bin/pyreverse -o dot -p RobotController -A -S *.py

# 2. Convertir en PNG
for dotfile in *.dot; do
    if [ -f "$dotfile" ]; then
        pngfile="${dotfile%.dot}.png"
        dot -Tpng "$dotfile" -o "$pngfile"
        echo "✅ Généré: $pngfile"
    fi
done

# 3. Générer aussi en SVG (plus net)
for dotfile in *.dot; do
    if [ -f "$dotfile" ]; then
        svgfile="${dotfile%.dot}.svg"
        dot -Tsvg "$dotfile" -o "$svgfile"
        echo "✅ Généré: $svgfile"
    fi
done

echo "📊 Fichiers générés:"
ls -la *.png *.svg *.dot
