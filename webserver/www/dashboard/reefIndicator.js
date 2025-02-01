
export class ReefIndicator {
    //////////////////////////////////////
    // Public Class methods
    //////////////////////////////////////
    constructor(draw_div_id, title, min_range, max_range, min_acceptable, max_acceptable) { 
        this.draw_div_id = draw_div_id;
        this.title = title;

        // State Variable Defaults
        this.hasData = false;
        this.curVal = 0;

        // Animation - intermediate drawn fill percentage
        // to keep stuff feeling smooth and high quality
        this.animatedCurValue = 0;
        this.prevTime = performance.now();

        // Set up drawing canvas within provided div
        this.canvas = document.createElement('canvas');
        this.docElem = document.getElementById(this.draw_div_id );
        this.canvas.id     = this.draw_div_id + "_canvas";
        this.docElem.appendChild(this.canvas);
        this.ctx = this.canvas.getContext("2d");
    }

    // Call this when NT is disconnected, or data is otherwise not available
    reportNoData(){
        this.hasData = false;
        this.curVal = 0;
    }

    // Call this whenever a new value for the widget is available.
    setVal(val) { 
        this.curVal = val;
        this.hasData = true;
    }

    //Call once per render loop to redraw the reef
    render() {

        this.recalcDrawConstants();
        
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        const centerX = this.canvas.width / 2;
        const centerY = this.canvas.height / 2;
        const indicatorSpacing = this.canvas.width / 10;
        const generalSeparationMargin = this.canvas.width / 20;
        const indicatorLength = this.canvas.width / 6;
        const reefRadius = this.canvas.height / 4; 
        const numSides = 6;
        const angleStep = (2 * Math.PI) / numSides;
        const indicatorsPerSide = 3;
        const totalIndicators = numSides * indicatorsPerSide;

        // Draw the reef
        this.ctx.beginPath();
        for (let i = 0; i < numSides; i++) {
            const angle = i * angleStep;
            const x = centerX + reefRadius * Math.cos(angle);
            const y = centerY + reefRadius * Math.sin(angle);
            if (i === 0) {
                this.ctx.moveTo(x, y);
            } else {
                this.ctx.lineTo(x, y);
            }
        }
        this.ctx.closePath();
        this.ctx.fillStyle = '#555555';
        this.ctx.fill();
        this.ctx.stroke();

        // Draw the indicators
        for (let i = 0; i < totalIndicators; i++) {
            const side = Math.floor(i / indicatorsPerSide);
            const position = i % indicatorsPerSide;
            const angle = side * angleStep;
            // rotate ctx draw frame to the side's angle, about the middle of the reef
            this.ctx.save();
            this.ctx.translate(centerX, centerY);
            this.ctx.rotate(angle);
            this.ctx.translate(-centerX, -centerY);


            //Since we've rotated, this math can be done assuming we're at the
            // "bottom" of the reef.
            const indicatorStartRadius = reefRadius + indicatorLength + generalSeparationMargin*2;
            const indicatorEndRadius = reefRadius + generalSeparationMargin;
            const x_0 = centerX + indicatorSpacing * (position - 1);
            const y_0 = centerY + indicatorStartRadius; 
            const x_1 = x_0
            const y_1 = centerY + indicatorEndRadius; 
            
            if(this.hasData && this.curVal === i){
                // yellow wide box under the selected indicator
                this.ctx.beginPath();
                this.ctx.moveTo(x_0, y_0);
                this.ctx.lineTo(x_1, y_1);
                // Set thickness to increase as widget width increases
                this.ctx.lineWidth = Math.max(1, this.canvas.width / 20);
                // Use the calculated color
                this.ctx.strokeStyle = "#BBBB00";
                this.ctx.stroke();
            }

            // Draw the actual indicator on top.
            var color;
            if (position === 1){
                //Algae
                color = '#008080';
            } else {
                //Coral
                color = '#B0B0B0';
            }

            this.ctx.beginPath();
            this.ctx.moveTo(x_0, y_0);
            this.ctx.lineTo(x_1, y_1);
            // Set thickness to increase as widget width increases
            this.ctx.lineWidth = Math.max(1, this.canvas.width / 100);
            // Use the calculated color
            this.ctx.strokeStyle = color;
            // stroke should end in an arrow
            this.ctx.lineCap = 'round';
            this.ctx.stroke();

            this.ctx.restore();
        }

        // Draw big red X if no data
        if (!this.hasData) {
            this.ctx.beginPath();
            this.ctx.moveTo(0,0);
            this.ctx.lineTo(this.canvas.width, this.canvas.height);
            this.ctx.moveTo(0, this.canvas.height);
            this.ctx.lineTo(this.canvas.width, 0);
            this.ctx.strokeStyle = 'red';
            this.ctx.lineWidth = 8;
            this.ctx.stroke();
        }
    }

    //////////////////////////////////////
    // Private, Helper methods
    //////////////////////////////////////

    recalcDrawConstants(){
        this.canvas.width  = this.docElem.offsetWidth;
        this.canvas.height = this.docElem.offsetHeight;

        //Drawing configurations

        this.gaugeCenterX = this.canvas.width/2;
        this.gaugeCenterY = this.canvas.height/1.9;

    }

    renderTextFixedSpacing(string, startX, startY){
        this.ctx.textAlign = 'center';
        this.ctx.textBaseline = 'middle';

        var spacing = this.valueTextSpacing;
        var totalLen = string.length * spacing;

        if(string[0] == '-'){
            totalLen -= spacing;
        }

        if(string.includes('.')){
            totalLen -= spacing*0.25;
        }

        var xPos = startX - totalLen/2;

        for(var idx=0; idx < string.length; idx++){
            var thisSpace = spacing;
            if(string[idx] == '.'){
                thisSpace *= 0.5;
            }
            xPos += thisSpace/2;
            this.ctx.fillText(string[idx], xPos, startY);
            xPos += thisSpace/2;
        }
    } 

  }