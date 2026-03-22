// Object Placement Mode - Allows users to add, move, and manage objects on the map

class ObjectPlacement {
    constructor(mapVisualizer) {
        this.mapVisualizer = mapVisualizer;
        this.isEnabled = false;
        this.placedObjects = [];
        this.selectedObject = null;
        this.isDragging = false;
        this.dragOffset = { x: 0, y: 0 };
        this.objectCounter = 0;
        
        // Grid snapping size in meters
        this.gridSize = 1.0;
        
        // Define available object types with their colors
        this.objectTypes = {
            'red_buoy': { label: 'Red Buoy', color: '#FF0000' },
            'green_buoy': { label: 'Green Buoy', color: '#00FF00' },
            'yellow_buoy': { label: 'Yellow Buoy', color: '#FFFF00' },
            'black_buoy': { label: 'Black Buoy', color: '#000000' },
            'red_pole_buoy': { label: 'Red Pole Buoy', color: '#CC0000' },
            'green_pole_buoy': { label: 'Green Pole Buoy', color: '#00CC00' },
            'dock': { label: 'Dock', color: '#666699' },
            'yellow_supply_drop': { label: 'Yellow Supply Drop', color: '#FFD700' },
            'black_supply_drop': { label: 'Black Supply Drop', color: '#333333' }
        };
        
        this.setupUI();
        this.setupCanvasEvents();
    }
    
    setupUI() {
        // Create and add the placement mode toggle button
        const header = document.querySelector('#header');
        const toggleButton = document.createElement('button');
        toggleButton.id = 'placement-mode-toggle';
        toggleButton.textContent = 'Enable Object Placement';
        toggleButton.className = 'placement-toggle-btn';
        
        toggleButton.addEventListener('click', () => {
            this.togglePlacementMode();
            toggleButton.textContent = this.isEnabled ? 'Disable Object Placement' : 'Enable Object Placement';
            toggleButton.classList.toggle('active');
        });
        
        header.appendChild(toggleButton);
        
        // Create the object toolbar
        const toolbar = document.createElement('div');
        toolbar.id = 'object-toolbar';
        toolbar.className = 'object-toolbar hidden';
        toolbar.innerHTML = '<h4>Place Objects</h4><div id="object-buttons-container"></div>';
        
        document.body.appendChild(toolbar);
        
        // Populate toolbar with object type buttons
        const container = document.getElementById('object-buttons-container');
        Object.entries(this.objectTypes).forEach(([typeKey, typeInfo]) => {
            const button = document.createElement('button');
            button.className = 'object-type-btn';
            button.draggable = true;
            button.dataset.objectType = typeKey;
            
            const colorBox = document.createElement('div');
            colorBox.className = 'object-color-box';
            colorBox.style.backgroundColor = typeInfo.color;
            
            const label = document.createElement('span');
            label.textContent = typeInfo.label;
            
            button.appendChild(colorBox);
            button.appendChild(label);
            
            // Setup drag start
            button.addEventListener('dragstart', (e) => {
                e.dataTransfer.effectAllowed = 'copy';
                e.dataTransfer.setData('text/plain', typeKey);
            });
            
            container.appendChild(button);
        });
    }
    
    setupCanvasEvents() {
        const canvas = this.mapVisualizer.canvas;
        
        // Allow dropping on canvas
        canvas.addEventListener('dragover', (e) => {
            if (this.isEnabled) {
                e.preventDefault();
                e.dataTransfer.dropEffect = 'copy';
            }
        });
        
        canvas.addEventListener('drop', (e) => {
            if (!this.isEnabled) return;
            e.preventDefault();
            
            const objectType = e.dataTransfer.getData('text/plain');
            if (!this.objectTypes[objectType]) return;
            
            // Get drop position in canvas coordinates
            const rect = canvas.getBoundingClientRect();
            const canvasX = e.clientX - rect.left;
            const canvasY = e.clientY - rect.top;
            
            // Convert to map coordinates
            const mapCoords = this.mapVisualizer.canvasToMap(canvasX, canvasY);
            
            // Create new placed object
            this.createPlacedObject(objectType, mapCoords.east, mapCoords.north);
        });
        
        // Handle canvas click for selection/deselection
        canvas.addEventListener('click', (e) => {
            if (!this.isEnabled) return;
            
            const rect = canvas.getBoundingClientRect();
            const canvasX = e.clientX - rect.left;
            const canvasY = e.clientY - rect.top;
            
            // Find if clicking on an object
            const clickedObject = this.getObjectAtPosition(canvasX, canvasY);
            
            if (clickedObject) {
                this.selectObject(clickedObject);
            } else {
                this.deselectObject();
            }
        });
        
        // Handle mouse down for dragging
        canvas.addEventListener('mousedown', (e) => {
            if (!this.isEnabled || !this.selectedObject) return;
            
            const rect = canvas.getBoundingClientRect();
            const canvasX = e.clientX - rect.left;
            const canvasY = e.clientY - rect.top;
            
            // Find object at click position
            const clickedObject = this.getObjectAtPosition(canvasX, canvasY);
            
            if (clickedObject && clickedObject === this.selectedObject) {
                this.isDragging = true;
                
                // Calculate offset between click position and object center
                const objCanvasPos = this.mapVisualizer.mapToCanvas(clickedObject.east, clickedObject.north);
                this.dragOffset.x = canvasX - objCanvasPos.x;
                this.dragOffset.y = canvasY - objCanvasPos.y;
            }
        });
        
        // Handle mouse move for dragging
        document.addEventListener('mousemove', (e) => {
            if (!this.isEnabled || !this.isDragging || !this.selectedObject) return;
            
            const rect = canvas.getBoundingClientRect();
            const canvasX = e.clientX - rect.left;
            const canvasY = e.clientY - rect.top;
            
            // Calculate new position accounting for offset
            const newCanvasX = canvasX - this.dragOffset.x;
            const newCanvasY = canvasY - this.dragOffset.y;
            
            // Convert back to map coordinates
            const mapCoords = this.mapVisualizer.canvasToMap(newCanvasX, newCanvasY);
            
            // Snap to grid
            const snappedCoords = this.snapToGrid(mapCoords.east, mapCoords.north);
            
            this.selectedObject.east = snappedCoords.east;
            this.selectedObject.north = snappedCoords.north;
            
            this.mapVisualizer.render();
        });
        
        // Handle mouse up to stop dragging
        document.addEventListener('mouseup', () => {
            this.isDragging = false;
        });
        
        // Handle keyboard delete key
        document.addEventListener('keydown', (e) => {
            if (!this.isEnabled || !this.selectedObject) return;
            
            if (e.key === 'Delete' || e.key === 'Backspace') {
                e.preventDefault();
                this.deletePlacedObject(this.selectedObject);
            }
        });
    }
    
    createPlacedObject(type, east, north) {
        const obj = {
            id: `placed_${this.objectCounter++}`,
            type: type,
            east: east,
            north: north,
            isPlaced: true
        };
        
        // Snap to grid on creation
        const snappedCoords = this.snapToGrid(obj.east, obj.north);
        obj.east = snappedCoords.east;
        obj.north = snappedCoords.north;
        
        this.placedObjects.push(obj);
        this.selectObject(obj);
        this.mapVisualizer.render();
    }
    
    snapToGrid(east, north) {
        // Snap coordinates to the nearest grid point
        const snappedEast = Math.round(east / this.gridSize) * this.gridSize;
        const snappedNorth = Math.round(north / this.gridSize) * this.gridSize;
        return { east: snappedEast, north: snappedNorth };
    }
    
    selectObject(obj) {
        this.deselectObject();
        this.selectedObject = obj;
        this.mapVisualizer.render();
    }
    
    deselectObject() {
        this.selectedObject = null;
        this.mapVisualizer.render();
    }
    
    deletePlacedObject(obj) {
        const index = this.placedObjects.indexOf(obj);
        if (index > -1) {
            this.placedObjects.splice(index, 1);
            this.deselectObject();
        }
    }
    
    getObjectAtPosition(canvasX, canvasY) {
        // Check placed objects in reverse order (most recent first)
        for (let i = this.placedObjects.length - 1; i >= 0; i--) {
            const obj = this.placedObjects[i];
            const objPos = this.mapVisualizer.mapToCanvas(obj.east, obj.north);
            
            // Check if click is within radius of object (8px + some tolerance)
            const distance = Math.sqrt(
                Math.pow(canvasX - objPos.x, 2) + 
                Math.pow(canvasY - objPos.y, 2)
            );
            
            if (distance <= 15) { // 15px tolerance
                return obj;
            }
        }
        return null;
    }
    
    togglePlacementMode() {
        this.isEnabled = !this.isEnabled;
        
        const toolbar = document.getElementById('object-toolbar');
        if (this.isEnabled) {
            toolbar.classList.remove('hidden');
        } else {
            toolbar.classList.add('hidden');
            this.deselectObject();
        }
        
        this.mapVisualizer.render();
    }
    
    drawPlacedObjects() {
        const ctx = this.mapVisualizer.ctx;
        
        for (const obj of this.placedObjects) {
            const pos = this.mapVisualizer.mapToCanvas(obj.east, obj.north);
            const objTypeInfo = this.objectTypes[obj.type];
            const color = objTypeInfo.color;
            
            // Draw circle
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(pos.x, pos.y, 8, 0, 2 * Math.PI);
            ctx.fill();
            
            // Draw outline (thicker if selected)
            ctx.strokeStyle = this.selectedObject === obj ? '#00FF00' : '#FFFFFF';
            ctx.lineWidth = this.selectedObject === obj ? 3 : 2;
            ctx.stroke();
            
            // Draw label
            ctx.fillStyle = '#000000';
            ctx.font = '10px Arial';
            const label = `${objTypeInfo.label}`;
            ctx.fillText(label, pos.x + 10, pos.y - 10);
        }
    }
    
    getPlacedObjects() {
        return this.placedObjects;
    }
    
    clearPlacedObjects() {
        this.placedObjects = [];
        this.deselectObject();
        this.mapVisualizer.render();
    }
}
