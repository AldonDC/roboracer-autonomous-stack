#!/bin/bash
# Script de sincronización automática QCar -> Assesment-Auto

SOURCE=~/Documents/qcar_remoto/Assesment_qcar_irs/
DEST=~/Documents/Assesment-Auto/physical_qcar/

echo "🔄 Sincronizando cambios del QCar físico..."

# Asegurar que la carpeta de destino existe
mkdir -p $DEST

# Sincronizar archivos omitiendo binarios y temporales
rsync -av --delete \
--exclude='build/' \
--exclude='install/' \
--exclude='log/' \
--exclude='.git/' \
--exclude='__pycache__/' \
--exclude='*.pyc' \
$SOURCE $DEST

echo "✅ Sincronización completada en $DEST"
