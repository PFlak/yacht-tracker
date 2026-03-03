const calendarContainer = document.getElementById('calendarContainer');
let currentDate = new Date();

// Pobranie wydarzeń z panelu accordion
function getEventsFromAccordion() {
    const events = {};
    document.querySelectorAll('.acc-item').forEach(item => {
        const boatName = item.querySelector('.acc-title').textContent;
        const lis = item.querySelectorAll('ul li');
        lis.forEach(li => {
            const text = li.textContent;
            const match = text.match(/\((\d{2})\.(\d{2})\.(\d{4})\)/);
            if(match){
                const day = match[1];
                const month = match[2];
                const year = match[3];
                const dateKey = `${year}-${month}-${day}`;
                if(!events[dateKey]) events[dateKey] = [];
                events[dateKey].push(`${boatName}: ${text.replace(match[0],'').trim()}`);
            }
        });
    });
    return events;
}

// Accordion po lewej 
document.querySelectorAll('.acc-item').forEach(item => {
    const title = item.querySelector('.acc-title');

    let icon = title.querySelector('.acc-icon');
    if(!icon){
        icon = document.createElement('span');
        icon.className = 'acc-icon';
        icon.textContent = '+';
        title.append(icon);
    }

    const content = item.querySelector('.acc-content');

    title.addEventListener('click', () => {
        const isOpen = content.style.maxHeight && content.style.maxHeight !== "0px";

        document.querySelectorAll('.acc-content').forEach(c => {
            if(c !== content){
                c.style.maxHeight = "0px";
                if(c.previousElementSibling){
                    const ic = c.previousElementSibling.querySelector('.acc-icon');
                    if(ic) ic.textContent = '+';
                }
                setTimeout(() => c.style.display = "none", 300);
            }
        });

        if(isOpen){
            content.style.maxHeight = "0px";
            icon.textContent = '+';
            setTimeout(() => content.style.display = "none", 300);
        } else {
            content.style.display = "block";
            const height = content.scrollHeight + "px";
            setTimeout(() => content.style.maxHeight = height, 10);
            icon.textContent = '×';
        }
    });
});


const events = getEventsFromAccordion();

// Render kalendarza
function renderCalendar() {
    calendarContainer.innerHTML = '';

    const month = currentDate.getMonth();
    const year = currentDate.getFullYear();

    // Header
    const header = document.createElement('div');
    header.className = 'calendar-header';

    const prevBtn = document.createElement('button');
    prevBtn.textContent = '<';
    prevBtn.onclick = () => { currentDate.setMonth(currentDate.getMonth() - 1); renderCalendar(); };

    const nextBtn = document.createElement('button');
    nextBtn.textContent = '>';
    nextBtn.onclick = () => { currentDate.setMonth(currentDate.getMonth() + 1); renderCalendar(); };

    const monthYear = document.createElement('div');
    monthYear.textContent = currentDate.toLocaleString('pl-PL', { month: 'long', year: 'numeric' });
    monthYear.style.cursor = 'pointer';
    monthYear.onclick = () => showYearDropdown(monthYear);

    header.appendChild(prevBtn);
    header.appendChild(monthYear);
    header.appendChild(nextBtn);
    calendarContainer.appendChild(header);

    // Dni tygodnia
    const daysRow = document.createElement('div');
    daysRow.className = 'calendar-days';
    ['Pon','Wt','Śr','Czw','Pt','Sob','Nd'].forEach(d => {
        const div = document.createElement('div');
        div.textContent = d;
        daysRow.appendChild(div);
    });
    calendarContainer.appendChild(daysRow);

    // Daty
    const datesDiv = document.createElement('div');
    datesDiv.className = 'calendar-dates';

    const firstDay = new Date(year, month, 1).getDay();
    const daysInMonth = new Date(year, month + 1, 0).getDate();
    let startDay = firstDay === 0 ? 6 : firstDay - 1;

    for(let i = 0; i < startDay; i++){
        datesDiv.appendChild(document.createElement('div'));
    }

    const today = new Date();

    for(let day=1; day<=daysInMonth; day++){
        const dayDiv = document.createElement('div');
        dayDiv.textContent = day;

        const dateKey = `${year}-${String(month+1).padStart(2,'0')}-${String(day).padStart(2,'0')}`;
        if(events[dateKey]){
            dayDiv.classList.add('event');
        }

        if(day === today.getDate() && month === today.getMonth() && year === today.getFullYear()){
            dayDiv.classList.add('today');
        }

        dayDiv.onclick = () => openPopup(day, month+1, year, events[dateKey]);
        datesDiv.appendChild(dayDiv);
    }

    calendarContainer.appendChild(datesDiv);
}

// Popup
function openPopup(day, month, year, dayEvents){
    const popup = document.getElementById('dayPopup');
    const popupDate = document.getElementById('popup-date');
    const popupEvent = document.getElementById('popup-event');

    popupDate.textContent = `${day}.${month}.${year}`;
    if(dayEvents && dayEvents.length > 0){
        popupEvent.innerHTML = '<ul>' + dayEvents.map(e => `<li>${e}</li>`).join('') + '</ul>';
    } else {
        popupEvent.textContent = 'Brak wydarzeń';
    }

    popup.style.display = 'flex';
}

function closePopup(){
    document.getElementById('dayPopup').style.display = 'none';
}

// Accordion po lewej
document.querySelectorAll('.acc-title').forEach(title => {
    title.addEventListener('click', () => {
        const content = title.nextElementSibling;

        if(content.style.maxHeight && content.style.maxHeight !== "0px"){
            content.style.maxHeight = "0px";
            setTimeout(() => content.style.display = "none", 300);
        } else {
            document.querySelectorAll('.acc-content').forEach(c => {
                if(c !== content){
                    c.style.maxHeight = "0px";
                    setTimeout(() => c.style.display = "none", 300);
                }
            });

            content.style.display = "block";
            const height = content.scrollHeight + "px";
            setTimeout(() => content.style.maxHeight = height, 10);
        }
    });
});

// Rozwijana lista lat
function showYearDropdown(monthYearDiv) {
    const existing = document.getElementById('yearDropdown');
    if(existing) existing.remove();

    const dropdown = document.createElement('div');
    dropdown.id = 'yearDropdown';
    dropdown.style.position = 'absolute';
    dropdown.style.top = monthYearDiv.getBoundingClientRect().bottom + window.scrollY + 'px';
    dropdown.style.left = monthYearDiv.getBoundingClientRect().left + 'px';
    dropdown.style.background = 'white';
    dropdown.style.border = '1px solid #ccc';
    dropdown.style.borderRadius = '5px';
    dropdown.style.maxHeight = '200px';
    dropdown.style.overflowY = 'scroll';
    dropdown.style.zIndex = 1000;
    dropdown.style.width = '120px';

    const currentYear = new Date().getFullYear();
    for(let y = currentYear - 50; y <= currentYear; y++){
        const item = document.createElement('div');
        item.textContent = y;
        item.style.padding = '5px';
        item.style.cursor = 'pointer';
        item.onmouseover = () => item.style.background = '#e0e0e0';
        item.onmouseout = () => item.style.background = 'white';
        item.onclick = () => {
            currentDate.setFullYear(y);
            dropdown.remove();
            renderCalendar();
        };
        dropdown.appendChild(item);
    }

    document.body.appendChild(dropdown);

    document.addEventListener('click', function handler(e){
        if(!dropdown.contains(e.target) && e.target !== monthYearDiv){
            dropdown.remove();
            document.removeEventListener('click', handler);
        }
    });
}

renderCalendar();
