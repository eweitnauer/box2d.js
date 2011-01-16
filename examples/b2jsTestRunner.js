(function(){

var Runner = function(canvas) {
	this._canvas = canvas;
	this._paused = true;
	this._fps = 200;
	this._dbgDraw = new b2DebugDraw();
	this._dbgDraw.m_drawScale = 8;
	this._dbgDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit || b2DebugDraw.e_centerOfMassBit);
	this.world = null;
}

Runner.prototype.log = function(arg) {
    if(typeof(window.console) != 'undefined') {
        console.log(arg);
    }
};

Runner.prototype.draw = function() {
	var c = this._canvas.getContext("2d");
	
	this._dbgDraw.SetSprite(c);
	if(this.world) {
		this.world.SetDebugDraw(this._dbgDraw);
		this.world.DrawDebugData();
	}
	
	c.fillStyle = "black";
	if(this._paused) {
		c.fillText("paused", 5, 15);
	} else {
		c.fillText("FPS: " + this._fpsAchieved, 5, 15);
	}
}

Runner.prototype.step = function(delta) {
	if(!this.world)
		return;
		
	this.world.ClearForces();
	
	var delta = (typeof delta == "undefined") ? 1/this._fps : delta;
	
	this.world.Step(delta, 10);	
}

Runner.prototype._update = function() {
	this.log("update");
	
	// derive passed time since last update. max. 10 secs
	var time = new Date().getTime();
	delta = (time - this._lastUpdate) / 1000;
	this._lastUpdate = time;
	if(delta > 10)
		delta = 1/this._fps;
		
	// see this._updateFPS
	this._fpsCounter++;
	
	this.step(delta);
	this.draw();
	if(!this._paused) {
		that = this;
		this._updateTimeout = window.setTimeout(function(){that._update()}, 1000/this._fps);
	}
}

Runner.prototype._updateFPS = function() {
	this._fpsAchieved = this._fpsCounter;
	this._fpsCounter = 0;
	
	if(!this._paused) {
		that = this;
		this._updateTimeout = window.setTimeout(function(){that._updateFPS()}, 1000);
	}
}

Runner.prototype.resume = function() {
	if(this._paused) {
		this._paused = false;
		this._lastUpdate = 0;
		this._update();
		this._updateFPS();
	}
}

Runner.prototype.pause = function() {
	this._paused = true;
	window.clearTimeout(this._updateTimeout);
	window.clearTimeout(this._updateFPSTimeput);
}

window.b2jsTestRunner = Runner;
	
})();