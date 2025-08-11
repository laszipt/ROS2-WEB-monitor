// parameters-service.js
// Fetch and display YAML parameter files in the Parameters section

(function(){
    const CONTAINER_ID = 'parameters-content';

    /** Escape HTML for safe display */
    function escapeHtml(text){
        const map = { '&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'": '&#039;' };
        return text.replace(/[&<>"']/g, m=>map[m]);
    }

    /** Fetch file list from backend */
    async function fetchFileList(){
        const resp = await fetch('/api/param-files');
        const data = await resp.json();
        if(data.error) throw new Error(data.error);
        return data.files || [];
    }

    /** Fetch content of one YAML file */
    async function fetchFileContent(relPath){
        const resp = await fetch(`/api/param-file?path=${encodeURIComponent(relPath)}`);
        if(!resp.ok){ throw new Error(await resp.text()); }
        return resp.text();
    }

    /** Show loading spinner */
    function showLoading(message='Loading...'){
        const container=document.getElementById(CONTAINER_ID);
        if(!container) return;
        container.innerHTML=`<div class="has-text-centered p-4"><div class="loader is-loading mb-3" style="height:50px;width:50px;margin:0 auto;"></div><p>${message}</p></div>`;
    }
    /** Hide loading, set HTML */
    function setContent(html){
        const container=document.getElementById(CONTAINER_ID);
        if(container) container.innerHTML=html;
    }
    /** Show error */
    function showError(msg){
        setContent(`<div class="notification is-danger"><strong>Error:</strong> ${escapeHtml(msg)}</div>`);
    }

    /** Render list and accordions */
    async function renderParameters(){
        showLoading('Searching for YAML parameter files...');
        try{
            const files=await fetchFileList();
            if(files.length===0){
                setContent('<div class="notification is-warning">No parameter YAML files found in <code>ros/src</code>.</div>');
                return;
            }
            injectStyle();
            const rows=files.map((f,i)=>{
                const arrow='▼';
                return `<tr class="param-summary" data-file="${encodeURIComponent(f)}">
                            <td class="param-name"><span class="toggle-arrow">${arrow}</span> ${f}</td>
                        </tr>
                        <tr class="param-details" style="display:none;"><td id="param-body-${i}"><p class="has-text-grey-light">Click to load...</p></td></tr>`;
            }).join('');
            const html=`<table class="table is-fullwidth is-striped is-hoverable is-narrow">
                            <thead><tr><th>Parameter File</th></tr></thead>
                            <tbody>${rows}</tbody>
                        </table>`;
            setContent(html);
            attachToggleHandlers();
        }catch(err){
            console.error('[Parameters] Error:',err);
            showError(err.message||err);
        }
    }

    /** Attach click listeners for accordions */
    function attachToggleHandlers(){
        document.querySelectorAll('.param-summary').forEach(row=>{
            row.addEventListener('click', async ()=>{
                const file=decodeURIComponent(row.getAttribute('data-file'));
                const detailsRow=row.nextElementSibling;
                const arrow=row.querySelector('.toggle-arrow');
                const open=detailsRow.style.display!=='' && detailsRow.style.display!=='none'?true:false;
                if(open){
                    detailsRow.style.display='none';
                    arrow.textContent='▼';
                    return;
                }
                const bodyCell=detailsRow.firstElementChild;
                if(!bodyCell.dataset.loaded){
                    bodyCell.innerHTML='<p>Loading...</p>';
                    try{
                        const txt=await fetchFileContent(file);
                        bodyCell.innerHTML=`<div class="field"><textarea class="textarea is-family-monospace" style="height:300px;font-size:1rem;">${escapeHtml(txt)}</textarea></div>
                        <div class="buttons mt-2">
                            <button class="button is-small is-success save-btn">Save</button>
                            <button class="button is-small is-info undo-btn">Undo</button>
                        </div>`;
                        const textarea=bodyCell.querySelector('textarea');
                        const saveBtn=bodyCell.querySelector('.save-btn');
                        const undoBtn=bodyCell.querySelector('.undo-btn');
                        let original=txt;
                        saveBtn.addEventListener('click',async()=>{
                            try{
                                const resp=await fetch(`/api/param-file?path=${encodeURIComponent(file)}`,{method:'POST',headers:{'Content-Type':'text/plain'},body:textarea.value});
                                const data=await resp.json();
                                if(data.error) throw new Error(data.error);
                                showNotification('File saved','success',3000);
                                original=textarea.value;
                            }catch(err){showNotification('Save failed: '+err.message,'danger',5000);}
                        });
                        undoBtn.addEventListener('click',()=>{
                            textarea.value=original;
                        });
                        bodyCell.dataset.loaded='1';
                    }catch(err){
                        bodyCell.innerHTML=`<p class="has-text-danger">Error: ${escapeHtml(err.message||err)}</p>`;
                    }
                }
                detailsRow.style.display='table-row';
                arrow.textContent='▲';
            });
        });
    }

    /** Inject styles once */
    function injectStyle(){
        const STYLE_ID='parameters-style';
        if(document.getElementById(STYLE_ID)) return;
        const style=document.createElement('style');
        style.id=STYLE_ID;
        style.textContent=`
            .param-name{max-width:400px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis;}
            .toggle-arrow{cursor:pointer; user-select:none; width:1em; display:inline-block; text-align:center;}
        `;
        document.head.appendChild(style);
    }

    // Expose global for ui-controller compatibility
    window.refreshParameterFiles = renderParameters;

    // Auto-render when Parameters section activated
    document.addEventListener('section-parameters-activated', ()=>{
        renderParameters();
    });
})();
